"""Protocol and runtime tests for the Group A/B TCP ground-station client."""

import json
import socket
import threading
import time
from pathlib import Path

import pytest

from uav_bt_runtime.bt import NodeStatus
from uav_bt_runtime.clock import ManualClock, SystemClock
from uav_bt_runtime.models import CommandEnvelope, CommandStatus
from uav_bt_runtime.package import load_mission_package
from uav_bt_runtime.task_testing import run_single_task
from uav_bt_runtime.tcp_transport import TcpCommandTransport


EXAMPLES = Path(__file__).resolve().parents[1] / "examples" / "joint_mission"


def _package(group_id):
    suffix = "a" if group_id == "GroupA" else "b"
    return load_mission_package(EXAMPLES / f"group_{suffix}", require_reviewed=False)


def _commands(package, plan_id):
    result = []
    for index, assignment in enumerate(
        package.context.plans[plan_id].robot_assignments.values()
    ):
        result.append(
            CommandEnvelope(
                mission_id=package.context.mission_id,
                action_instance_id=f"TestAction:{plan_id}",
                command_id=f"runtime-command-{index}",
                group_id=package.context.group_id,
                robot_id=assignment.robot_id,
                command_type=assignment.command_type,
                payload={**assignment.payload, "_plan_id": plan_id},
                timeout_s=300.0,
                issued_at=100.0,
                expires_at=400.0,
                sequence_number=index,
            )
        )
    return tuple(result)


def _transport_for_message(package):
    transport = object.__new__(TcpCommandTransport)
    transport.package = package
    transport.group_id = package.context.group_id
    return transport


def _recv_json_line(connection):
    buffer = b""
    while b"\n" not in buffer:
        chunk = connection.recv(4096)
        if not chunk:
            raise RuntimeError("connection closed before a complete JSON line")
        buffer += chunk
    return json.loads(buffer.split(b"\n", 1)[0].decode("utf-8"))


class MockGroundStation:
    """Small protocol peer used only by runtime tests."""

    def __init__(self, group_id, terminal_status="COMPLETED", failed_uavs=None):
        self.group_id = group_id
        self.terminal_status = terminal_status
        self.failed_uavs = list(failed_uavs or [])
        self.received = []
        self.dispatched = []
        self.holds = []
        self._cache = {}
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._listener = None
        self._client = None
        self.port = 0
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def start(self):
        self._thread.start()
        assert self._ready.wait(1.0)
        return self

    def close(self):
        self._stop.set()
        for connection in (self._client, self._listener):
            if connection is not None:
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                connection.close()
        self._thread.join(timeout=1.0)

    def _send(self, message):
        if self._client is not None:
            self._client.sendall(
                json.dumps(message, separators=(",", ":")).encode("utf-8") + b"\n"
            )

    def _status(self, command, status, **extra):
        message = {
            "version": "1.0",
            "type": "STATUS",
            "mission_id": command["mission_id"],
            "group_id": self.group_id,
            "command_id": command["command_id"],
            "status": status,
        }
        message.update(extra)
        return message

    def _handle(self, command):
        self.received.append(command)
        command_id = command["command_id"]
        if command_id in self._cache:
            self._send(self._cache[command_id])
            return
        if command.get("group_id") != self.group_id:
            failed = self._status(
                command,
                "FAILED",
                failed_uavs=[item["uav_id"] for item in command["assignments"]],
                error_code="WRONG_GROUP",
                message="command sent to the wrong ground station",
            )
            self._cache[command_id] = failed
            self._send(failed)
            return
        if command["command"] == "HOLD":
            self.holds.append(command)
            completed = self._status(
                command,
                "COMPLETED",
                completed_uavs=[item["uav_id"] for item in command["assignments"]],
            )
            self._cache[command_id] = completed
            self._send(completed)
            return

        self.dispatched.append(command)
        accepted = self._status(command, "ACCEPTED")
        self._cache[command_id] = accepted
        self._send(accepted)
        if self.terminal_status is None:
            return
        if self.terminal_status == "COMPLETED":
            terminal = self._status(
                command,
                "COMPLETED",
                completed_uavs=[item["uav_id"] for item in command["assignments"]],
            )
        else:
            terminal = self._status(
                command,
                "FAILED",
                failed_uavs=self.failed_uavs,
                error_code="TEST_GROUND_FAILURE",
                message="injected ground-station failure",
            )
        self._cache[command_id] = terminal
        self._send(terminal)

    def _serve(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        listener.settimeout(0.1)
        self._listener = listener
        self.port = listener.getsockname()[1]
        self._ready.set()
        try:
            while not self._stop.is_set():
                try:
                    client, _ = listener.accept()
                    break
                except socket.timeout:
                    continue
            else:
                return
            self._client = client
            client.settimeout(0.1)
            buffer = bytearray()
            while not self._stop.is_set():
                try:
                    chunk = client.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    return
                if not chunk:
                    return
                buffer.extend(chunk)
                while b"\n" in buffer:
                    raw, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    if raw.strip():
                        self._handle(json.loads(raw.decode("utf-8")))
        finally:
            try:
                listener.close()
            except OSError:
                pass


@pytest.mark.parametrize(
    "plan_id,command_type,robot_count",
    [
        ("prepare-a-two-uav", "MOVE_TO", 2),
        ("prepare-a", "MOVE_TO", 12),
        ("coverage-segment-1", "FOLLOW_ROUTE", 12),
        ("fault-exit", "FAULT_EXIT", 6),
        ("recovery-a", "MOVE_TO", 9),
        ("coverage-segment-2", "FOLLOW_ROUTE", 9),
        ("hold-a-end", "HOVER", 9),
    ],
)
def test_group_a_plan_maps_to_one_group_message(plan_id, command_type, robot_count):
    package = _package("GroupA")
    commands = _commands(package, plan_id)
    message = _transport_for_message(package)._build_message(commands, "a-command")

    assert message["group_id"] == "GroupA"
    assert message["command"] == command_type
    assert len(message["assignments"]) == robot_count
    assert {item["uav_id"] for item in message["assignments"]} == {
        item.robot_id for item in commands
    }


def test_group_a_follow_route_contains_leader_route_and_follower_markers():
    package = _package("GroupA")
    message = _transport_for_message(package)._build_message(
        _commands(package, "coverage-segment-1"), "coverage-command"
    )
    by_id = {item["uav_id"]: item for item in message["assignments"]}

    assert message["leader_id"] == "A01"
    assert len(by_id["A01"]["waypoints"]) == 11
    assert all(
        item == {"uav_id": robot_id, "formation_follow": True}
        for robot_id, item in by_id.items()
        if robot_id != "A01"
    )


def test_group_a_fault_exit_keeps_each_aircraft_route():
    package = _package("GroupA")
    message = _transport_for_message(package)._build_message(
        _commands(package, "fault-exit"), "fault-exit-command"
    )
    assert all(len(item["waypoints"]) == 2 for item in message["assignments"])


def test_group_a_primary_plans_have_no_implicit_hover():
    package = _package("GroupA")
    for plan_id in (
        "prepare-a",
        "coverage-segment-1",
        "recovery-a",
        "coverage-segment-2",
    ):
        assert all(
            assignment.terminal_action is None
            for assignment in package.context.plans[plan_id].robot_assignments.values()
        )
    assert {
        item.command_type
        for item in package.context.plans["hold-a-end"].robot_assignments.values()
    } == {"HOVER"}


def test_group_b_attack_and_return_keep_existing_assignments_with_group_id():
    package = _package("GroupB")
    transport = _transport_for_message(package)
    attack = transport._build_message(_commands(package, "strike-targets"), "attack")
    returned = transport._build_message(
        _commands(package, "return-strike-uavs"), "return"
    )

    assert attack["group_id"] == returned["group_id"] == "GroupB"
    assert attack["command"] == "ATTACK"
    assert {item["target_id"] for item in attack["assignments"]} == {
        "target-1",
        "target-2",
    }
    assert returned["command"] == "RETURN"
    assert returned["assignments"] == [
        {
            **assignment.payload["waypoints"][0],
            "uav_id": assignment.robot_id,
        }
        for assignment in package.context.plans[
            "return-strike-uavs"
        ].robot_assignments.values()
    ]


def test_group_b_b01_attack_test_uses_one_assignment():
    package = _package("GroupB")
    transport = _transport_for_message(package)
    commands = tuple(_commands(package, "strike-target-b01-test"))

    transport._validate_batch(commands)
    message = transport._build_message(commands, "b01-attack-test")

    assert message["command"] == "ATTACK"
    assert message["assignments"] == [
        {
            "x": package.world.targets["target-1"]["x"],
            "y": package.world.targets["target-1"]["y"],
            "z": package.world.targets["target-1"]["z"],
            "yaw": package.world.targets["target-1"]["yaw"],
            "uav_id": "B01",
            "target_id": "target-1",
        }
    ]


@pytest.mark.parametrize(
    "group_id,plan_id,task_name",
    [
        ("GroupA", "prepare-a-two-uav", "prepare-two-uav"),
        ("GroupA", "prepare-a", "prepare"),
        ("GroupB", "strike-targets", "strike-targets"),
    ],
)
def test_behavior_tree_task_advances_on_tcp_completed(group_id, plan_id, task_name):
    package = _package(group_id)
    server = MockGroundStation(group_id).start()
    transport = TcpCommandTransport(package, SystemClock(), "127.0.0.1", server.port)
    try:
        transport.wait_until_ready()
        result = run_single_task(
            package,
            task_name,
            transport,
            transport.clock,
            max_ticks=300,
            wait_for_next_tick=lambda: time.sleep(0.005),
        )
        assert result.status == NodeStatus.SUCCESS
        assert len(server.dispatched) == 1
        assert server.dispatched[0]["command"] == _commands(package, plan_id)[0].command_type
    finally:
        transport.close()
        server.close()


def test_failed_aircraft_is_preserved_in_runtime_feedback():
    package = _package("GroupB")
    server = MockGroundStation("GroupB", "FAILED", ["B02"]).start()
    transport = TcpCommandTransport(package, SystemClock(), "127.0.0.1", server.port)
    try:
        transport.wait_until_ready()
        result = run_single_task(
            package,
            "strike-targets",
            transport,
            transport.clock,
            max_ticks=300,
            wait_for_next_tick=lambda: time.sleep(0.005),
        )
        assert result.status == NodeStatus.FAILURE
        assert result.failed_robot_ids == ("B02",)
    finally:
        transport.close()
        server.close()


def _start_one_shot_server(handler):
    ready = threading.Event()
    port_holder = []

    def run():
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port_holder.append(listener.getsockname()[1])
        ready.set()
        client, _ = listener.accept()
        try:
            handler(client)
        finally:
            client.close()
            listener.close()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    assert ready.wait(1.0)
    return port_holder[0], thread


def test_missing_accepted_status_times_out_after_five_seconds():
    received = threading.Event()

    def silent_server(client):
        _recv_json_line(client)
        received.set()
        time.sleep(0.5)

    port, thread = _start_one_shot_server(silent_server)
    package = _package("GroupA")
    clock = ManualClock()
    transport = TcpCommandTransport(
        package, clock, "127.0.0.1", port, accept_timeout_s=5.0
    )
    try:
        transport.wait_until_ready()
        transport.send_batch(_commands(package, "prepare-a"))
        assert received.wait(1.0)
        clock.advance(5.0)
        feedback = transport.poll_feedback()
        assert len(feedback) == 12
        assert {item.status for item in feedback} == {CommandStatus.TIMEOUT}
        assert {item.error_code for item in feedback} == {"ACCEPT_TIMEOUT"}
    finally:
        transport.close()
        thread.join(timeout=1.0)


def test_connection_loss_fails_the_active_batch():
    port, thread = _start_one_shot_server(lambda client: _recv_json_line(client))
    package = _package("GroupB")
    transport = TcpCommandTransport(package, SystemClock(), "127.0.0.1", port)
    try:
        transport.wait_until_ready()
        transport.send_batch(_commands(package, "strike-targets"))
        deadline = time.monotonic() + 2.0
        feedback = []
        while time.monotonic() < deadline and not feedback:
            feedback.extend(transport.poll_feedback())
            time.sleep(0.005)
        assert len(feedback) == 2
        assert {item.status for item in feedback} == {CommandStatus.FAILURE}
        assert {item.error_code for item in feedback} == {"TCP_DISCONNECTED"}
    finally:
        transport.close()
        thread.join(timeout=1.0)


def test_wrong_group_status_fails_closed():
    def wrong_group_server(client):
        command = _recv_json_line(client)
        client.sendall(
            json.dumps(
                {
                    "version": "1.0",
                    "type": "STATUS",
                    "mission_id": command["mission_id"],
                    "group_id": "GroupB",
                    "command_id": command["command_id"],
                    "status": "ACCEPTED",
                }
            ).encode("utf-8")
            + b"\n"
        )
        time.sleep(0.1)

    port, thread = _start_one_shot_server(wrong_group_server)
    package = _package("GroupA")
    transport = TcpCommandTransport(package, SystemClock(), "127.0.0.1", port)
    try:
        transport.wait_until_ready()
        transport.send_batch(_commands(package, "prepare-a"))
        deadline = time.monotonic() + 2.0
        feedback = []
        while time.monotonic() < deadline and not feedback:
            feedback.extend(transport.poll_feedback())
            time.sleep(0.005)
        assert len(feedback) == 12
        assert {item.status for item in feedback} == {CommandStatus.FAILURE}
        assert {item.error_code for item in feedback} == {
            "INVALID_GROUND_STATION_STATUS"
        }
    finally:
        transport.close()
        thread.join(timeout=1.0)


def test_mock_ground_station_deduplicates_command_id():
    server = MockGroundStation("GroupB", terminal_status=None).start()
    client = socket.create_connection(("127.0.0.1", server.port), timeout=1.0)
    command = _transport_for_message(_package("GroupB"))._build_message(
        _commands(_package("GroupB"), "return-strike-uavs"), "duplicate-command"
    )
    payload = json.dumps(command).encode("utf-8") + b"\n"
    try:
        client.sendall(payload)
        assert _recv_json_line(client)["status"] == "ACCEPTED"
        client.sendall(payload)
        assert _recv_json_line(client)["status"] == "ACCEPTED"
        assert len(server.dispatched) == 1
    finally:
        client.close()
        server.close()


@pytest.mark.parametrize(
    "server_group,command_group,plan_id",
    [
        ("GroupB", "GroupA", "prepare-a"),
        ("GroupA", "GroupB", "strike-targets"),
    ],
)
def test_ground_station_rejects_command_for_the_other_group(
    server_group, command_group, plan_id
):
    server = MockGroundStation(server_group, terminal_status=None).start()
    client = socket.create_connection(("127.0.0.1", server.port), timeout=1.0)
    package = _package(command_group)
    command = _transport_for_message(package)._build_message(
        _commands(package, plan_id), "wrong-group-command"
    )
    try:
        client.sendall(json.dumps(command).encode("utf-8") + b"\n")
        response = _recv_json_line(client)
        assert response["status"] == "FAILED"
        assert response["error_code"] == "WRONG_GROUP"
        assert not server.dispatched
    finally:
        client.close()
        server.close()


def test_emergency_hold_carries_group_and_active_aircraft():
    package = _package("GroupA")
    server = MockGroundStation("GroupA", terminal_status=None).start()
    transport = TcpCommandTransport(package, SystemClock(), "127.0.0.1", server.port)
    try:
        transport.wait_until_ready()
        transport.send_batch(_commands(package, "fault-exit"))
        deadline = time.monotonic() + 1.0
        while not server.dispatched and time.monotonic() < deadline:
            time.sleep(0.005)
        transport.emergency_hold(package.context.mission_id)
        while not server.holds and time.monotonic() < deadline:
            time.sleep(0.005)
        assert server.holds[0]["group_id"] == "GroupA"
        assert {item["uav_id"] for item in server.holds[0]["assignments"]} == {
            f"A{index:02d}" for index in range(7, 13)
        }
    finally:
        transport.close()
        server.close()
