"""Group-level TCP transport for physical Group A and Group B ground stations."""

from __future__ import annotations

import json
import socket
import threading
import uuid
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from .clock import Clock
from .models import CommandEnvelope, CommandFeedback, CommandStatus


PROTOCOL_VERSION = "1.0"
MAX_MESSAGE_BYTES = 64 * 1024
SUPPORTED_COMMANDS = {
    "GroupA": {"MOVE_TO", "FOLLOW_ROUTE", "FAULT_EXIT", "HOVER"},
    "GroupB": {"ATTACK", "RETURN"},
}
GROUP_B_SINGLE_UAV_TEST_PLAN = "strike-target-b01-test"


@dataclass
class _ActiveBatch:
    wire_command_id: str
    commands: Tuple[CommandEnvelope, ...]
    sent_at: float
    accepted: bool = False


@dataclass(frozen=True)
class TcpCommandRecord:
    """One Group-level command sent over the TCP connection."""

    command_id: str
    command: str
    robot_ids: Tuple[str, ...]
    message: Mapping[str, object]

    def to_dict(self) -> Dict[str, object]:
        return {
            "command_id": self.command_id,
            "command": self.command,
            "robot_ids": list(self.robot_ids),
            "message": dict(self.message),
        }


class TcpCommandTransport:
    """Aggregate one Group task batch into one newline-delimited JSON message."""

    def __init__(
        self,
        package,
        clock: Clock,
        host: str,
        port: int,
        connect_timeout_s: float = 10.0,
        accept_timeout_s: float = 5.0,
    ) -> None:
        if package.context.group_id not in SUPPORTED_COMMANDS:
            raise ValueError("TCP transport only supports GroupA and GroupB")
        if not host:
            raise ValueError("TCP host cannot be empty")
        if not 1 <= int(port) <= 65535:
            raise ValueError("TCP port must be between 1 and 65535")
        if connect_timeout_s <= 0 or accept_timeout_s <= 0:
            raise ValueError("TCP connect and accept timeouts must be positive")

        self.package = package
        self.group_id = package.context.group_id
        self.clock = clock
        self.host = host
        self.port = int(port)
        self.connect_timeout_s = float(connect_timeout_s)
        self.accept_timeout_s = float(accept_timeout_s)

        self._lock = threading.RLock()
        self._send_lock = threading.Lock()
        self._socket: Optional[socket.socket] = None
        self._reader: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._pending_feedback: List[CommandFeedback] = []
        self._active: Optional[_ActiveBatch] = None
        self._connection_error: Optional[str] = None
        self.history: List[TcpCommandRecord] = []
        self.closed = False

    def wait_until_ready(self) -> None:
        """Open the persistent TCP connection to this Group's ground station."""

        with self._lock:
            if self.closed:
                raise RuntimeError("TCP command transport is closed")
            if self._socket is not None:
                return
        try:
            connection = socket.create_connection(
                (self.host, self.port), timeout=self.connect_timeout_s
            )
            connection.settimeout(0.5)
        except OSError as exc:
            raise RuntimeError(
                f"cannot connect to {self.group_id} ground station "
                f"{self.host}:{self.port}: {exc}"
            ) from exc

        with self._lock:
            if self.closed:
                connection.close()
                raise RuntimeError("TCP command transport is closed")
            self._socket = connection
            self._connection_error = None
            self._stop.clear()
            self._reader = threading.Thread(
                target=self._reader_loop,
                name=f"{self.group_id.lower()}-tcp-status-reader",
                daemon=True,
            )
            self._reader.start()

    def send(self, command: CommandEnvelope) -> None:
        if command.command_type == "SAFE_HOLD":
            self.emergency_hold(command.mission_id)
            return
        self.send_batch([command])

    def send_batch(self, commands: Sequence[CommandEnvelope]) -> None:
        command_batch = tuple(commands)
        self._validate_batch(command_batch)
        wire_command_id = f"tcp-{uuid.uuid4().hex}"
        message = self._build_message(command_batch, wire_command_id)

        with self._lock:
            if self.closed:
                raise RuntimeError("TCP command transport is closed")
            if self._socket is None:
                raise RuntimeError("TCP command transport is not connected")
            if self._connection_error is not None:
                raise RuntimeError(self._connection_error)
            if self._active is not None:
                raise RuntimeError(
                    f"the {self.group_id} ground station already has an active command"
                )
            active = _ActiveBatch(
                wire_command_id=wire_command_id,
                commands=command_batch,
                sent_at=self.clock.monotonic(),
            )
            self._active = active

        try:
            self._send_json(message)
        except Exception:
            with self._lock:
                if self._active is active:
                    self._active = None
            raise

        with self._lock:
            self.history.append(
                TcpCommandRecord(
                    command_id=wire_command_id,
                    command=command_batch[0].command_type,
                    robot_ids=tuple(item.robot_id for item in command_batch),
                    message=message,
                )
            )

    def cancel(self, command_id: str) -> None:
        with self._lock:
            active = self._active
            if active is None or command_id not in {
                item.command_id for item in active.commands
            }:
                return
            mission_id = active.commands[0].mission_id
            robot_ids = tuple(item.robot_id for item in active.commands)
        self._send_hold(mission_id, robot_ids, "command cancelled")
        self._finish_active(CommandStatus.CANCELLED, "COMMAND_CANCELLED", "command cancelled")

    def emergency_hold(self, mission_id: str) -> None:
        with self._lock:
            active = self._active
            robot_ids = (
                tuple(item.robot_id for item in active.commands)
                if active is not None
                else tuple(self.package.context.roster.active_ids)
            )
        self._send_hold(mission_id, robot_ids, "runtime requested safe hold")
        if active is not None:
            self._finish_active(
                CommandStatus.CANCELLED,
                "EMERGENCY_HOLD",
                "runtime requested safe hold",
            )

    def poll_feedback(self) -> List[CommandFeedback]:
        with self._lock:
            active = self._active
            if (
                active is not None
                and not active.accepted
                and self.clock.monotonic() - active.sent_at >= self.accept_timeout_s
            ):
                timed_out = active
            else:
                timed_out = None

        if timed_out is not None:
            try:
                self._send_hold(
                    timed_out.commands[0].mission_id,
                    tuple(item.robot_id for item in timed_out.commands),
                    "ACCEPTED timeout",
                )
            except RuntimeError:
                pass
            self._finish_active(
                CommandStatus.TIMEOUT,
                "ACCEPT_TIMEOUT",
                f"ground station did not reply ACCEPTED within "
                f"{self.accept_timeout_s:g} seconds",
            )

        with self._lock:
            feedback = list(self._pending_feedback)
            self._pending_feedback.clear()
        return feedback

    def close(self) -> None:
        with self._lock:
            if self.closed:
                return
            self.closed = True
            connection = self._socket
            reader = self._reader
            self._socket = None
            self._stop.set()
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=1.0)

    def _validate_batch(self, commands: Tuple[CommandEnvelope, ...]) -> None:
        if not commands:
            raise ValueError("TCP command batch cannot be empty")
        common = {
            (
                item.mission_id,
                item.action_instance_id,
                item.group_id,
                item.command_type,
            )
            for item in commands
        }
        if len(common) != 1:
            raise ValueError(
                "one TCP message requires a single mission, Action instance, Group, and command type"
            )
        if commands[0].group_id != self.group_id:
            raise ValueError(
                f"transport for {self.group_id} cannot send another Group's command"
            )
        supported = SUPPORTED_COMMANDS[self.group_id]
        if commands[0].command_type not in supported:
            raise ValueError(
                f"unsupported {self.group_id} TCP command "
                f"{commands[0].command_type!r}"
            )
        robot_ids = [item.robot_id for item in commands]
        if len(robot_ids) != len(set(robot_ids)):
            raise ValueError("one TCP command cannot contain duplicate UAV IDs")
        if self.group_id == "GroupB" and set(robot_ids) != {"B01", "B02"}:
            plan_ids = {item.payload.get("_plan_id") for item in commands}
            is_b01_test = (
                commands[0].command_type == "ATTACK"
                and robot_ids == ["B01"]
                and plan_ids == {GROUP_B_SINGLE_UAV_TEST_PLAN}
            )
            if not is_b01_test:
                raise ValueError(
                    "Group B ATTACK and RETURN require exactly B01 and B02; "
                    "only strike-target-b01-test may send B01 alone"
                )

    def _build_message(
        self, commands: Tuple[CommandEnvelope, ...], wire_command_id: str
    ) -> Dict[str, object]:
        command_type = commands[0].command_type
        assignments = [self._assignment(item) for item in commands]
        message: Dict[str, object] = {
            "version": PROTOCOL_VERSION,
            "type": "COMMAND",
            "mission_id": commands[0].mission_id,
            "group_id": self.group_id,
            "command_id": wire_command_id,
            "command": command_type,
            "timeout_s": min(float(item.timeout_s) for item in commands),
            "assignments": assignments,
        }
        if command_type == "FOLLOW_ROUTE":
            leaders = [
                item.robot_id
                for item in commands
                if self.package.context.uav_specs[item.robot_id].is_leader
            ]
            if len(leaders) != 1:
                raise ValueError("FOLLOW_ROUTE requires exactly one configured Leader")
            message["leader_id"] = leaders[0]
        return message

    def _assignment(self, command: CommandEnvelope) -> Dict[str, object]:
        command_type = command.command_type
        if command_type == "ATTACK":
            return self._attack_assignment(command)
        if command_type == "RETURN":
            return self._single_waypoint_assignment(command, "RETURN")
        if command_type == "MOVE_TO":
            target_pose = command.payload.get("target_pose")
            if not isinstance(target_pose, Mapping):
                raise ValueError("MOVE_TO requires target_pose")
            assignment = self._pose_fields(target_pose)
            assignment["uav_id"] = command.robot_id
            return assignment
        if command_type in {"FOLLOW_ROUTE", "FAULT_EXIT"}:
            waypoints = command.payload.get("waypoints")
            if isinstance(waypoints, list) and waypoints:
                return {
                    "uav_id": command.robot_id,
                    "waypoints": [self._pose_fields(item) for item in waypoints],
                }
            if command_type == "FOLLOW_ROUTE" and command.payload.get(
                "formation_follow"
            ) is True:
                return {"uav_id": command.robot_id, "formation_follow": True}
            raise ValueError(f"{command_type} requires non-empty waypoints")
        if command_type == "HOVER":
            return {"uav_id": command.robot_id}
        raise ValueError(f"unsupported {self.group_id} TCP command {command_type!r}")

    def _attack_assignment(self, command: CommandEnvelope) -> Dict[str, object]:
        target_id = command.payload.get("target_id")
        if not isinstance(target_id, str) or not target_id:
            raise ValueError("ATTACK requires target_id")
        target = self.package.world.targets.get(target_id)
        if not isinstance(target, Mapping):
            raise ValueError(f"ATTACK references unknown target {target_id!r}")
        assignment = self._pose_fields(target)
        assignment.update({"uav_id": command.robot_id, "target_id": target_id})
        return assignment

    def _single_waypoint_assignment(
        self, command: CommandEnvelope, command_type: str
    ) -> Dict[str, object]:
        waypoints = command.payload.get("waypoints")
        if not isinstance(waypoints, list) or len(waypoints) != 1:
            raise ValueError(
                f"{command_type} requires exactly one waypoint per UAV"
            )
        waypoint = waypoints[0]
        if not isinstance(waypoint, Mapping):
            raise ValueError("RETURN waypoint must be an object")
        assignment = self._pose_fields(waypoint)
        assignment["uav_id"] = command.robot_id
        return assignment

    @staticmethod
    def _pose_fields(value: Mapping[str, object]) -> Dict[str, object]:
        try:
            return {
                "x": float(value["x"]),
                "y": float(value["y"]),
                "z": float(value["z"]),
                "yaw": float(value.get("yaw", 0.0)),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("assignment requires numeric x, y, and z") from exc

    def _send_hold(
        self, mission_id: str, robot_ids: Sequence[str], reason: str
    ) -> None:
        self._send_json(
            {
                "version": PROTOCOL_VERSION,
                "type": "COMMAND",
                "mission_id": mission_id,
                "group_id": self.group_id,
                "command_id": f"hold-{uuid.uuid4().hex}",
                "command": "HOLD",
                "timeout_s": 5.0,
                "assignments": [{"uav_id": item} for item in robot_ids],
                "reason": reason,
            }
        )

    def _send_json(self, message: Mapping[str, object]) -> None:
        payload = json.dumps(
            message, ensure_ascii=False, separators=(",", ":"), allow_nan=False
        ).encode("utf-8") + b"\n"
        if len(payload) > MAX_MESSAGE_BYTES:
            raise ValueError("TCP JSON message exceeds 64 KiB")
        with self._send_lock:
            with self._lock:
                connection = self._socket
                connection_error = self._connection_error
            if connection is None or connection_error is not None:
                raise RuntimeError(connection_error or "TCP command transport is not connected")
            try:
                connection.sendall(payload)
            except OSError as exc:
                self._mark_disconnected(str(exc))
                raise RuntimeError(f"failed to send TCP command: {exc}") from exc

    def _reader_loop(self) -> None:
        buffer = bytearray()
        while not self._stop.is_set():
            with self._lock:
                connection = self._socket
            if connection is None:
                return
            try:
                chunk = connection.recv(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    self._mark_disconnected(str(exc))
                return
            if not chunk:
                if not self._stop.is_set():
                    self._mark_disconnected("ground station closed the TCP connection")
                return
            buffer.extend(chunk)
            if len(buffer) > MAX_MESSAGE_BYTES and b"\n" not in buffer:
                self._mark_disconnected(
                    "ground station sent a message larger than 64 KiB"
                )
                return
            while b"\n" in buffer:
                raw, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)
                if not raw.strip():
                    continue
                try:
                    decoded = json.loads(raw.decode("utf-8"))
                    self._handle_status(decoded)
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                    self._finish_active(
                        CommandStatus.FAILURE,
                        "INVALID_GROUND_STATION_STATUS",
                        str(exc),
                    )

    def _handle_status(self, message: object) -> None:
        if not isinstance(message, dict):
            raise ValueError("ground-station status must be a JSON object")
        if message.get("version") != PROTOCOL_VERSION or message.get("type") != "STATUS":
            raise ValueError("ground-station status has an unsupported version or type")
        if message.get("group_id") != self.group_id:
            raise ValueError("ground-station status has the wrong group_id")
        status = message.get("status")
        if status not in {"ACCEPTED", "COMPLETED", "FAILED"}:
            raise ValueError(
                f"ground station returned unsupported status {status!r}"
            )

        with self._lock:
            active = self._active
            if active is None:
                return
            if (
                message.get("mission_id") != active.commands[0].mission_id
                or message.get("command_id") != active.wire_command_id
            ):
                return

        if status == "ACCEPTED":
            with self._lock:
                if self._active is not active or active.accepted:
                    return
                active.accepted = True
                for command in active.commands:
                    self._append_feedback(command, CommandStatus.ACCEPTED)
                    self._append_feedback(command, CommandStatus.RUNNING)
            return

        expected = {item.robot_id for item in active.commands}
        if status == "COMPLETED":
            completed = message.get("completed_uavs")
            if (
                not isinstance(completed, list)
                or len(completed) != len(expected)
                or set(completed) != expected
            ):
                raise ValueError("COMPLETED must list every commanded UAV exactly once")
            self._finish_active(CommandStatus.SUCCESS)
            return

        failed_value = message.get("failed_uavs")
        failed = set(failed_value) if isinstance(failed_value, list) else set()
        if (
            not failed
            or len(failed) != len(failed_value)
            or not failed <= expected
        ):
            raise ValueError("FAILED contains invalid failed_uavs")
        self._finish_failed_active(
            failed,
            str(message.get("error_code") or "BOTTOM_TASK_FAILED"),
            str(message.get("message") or f"{self.group_id} task failed"),
        )

    def _finish_active(
        self,
        status: CommandStatus,
        error_code: Optional[str] = None,
        error_message: Optional[str] = None,
    ) -> None:
        with self._lock:
            active = self._active
            if active is None:
                return
            self._active = None
            for command in active.commands:
                self._append_feedback(command, status, error_code, error_message)

    def _finish_failed_active(
        self, failed_ids: set, error_code: str, error_message: str
    ) -> None:
        with self._lock:
            active = self._active
            if active is None:
                return
            self._active = None
            for command in active.commands:
                if command.robot_id in failed_ids:
                    self._append_feedback(
                        command, CommandStatus.FAILURE, error_code, error_message
                    )
                else:
                    self._append_feedback(command, CommandStatus.SUCCESS)

    def _append_feedback(
        self,
        command: CommandEnvelope,
        status: CommandStatus,
        error_code: Optional[str] = None,
        error_message: Optional[str] = None,
    ) -> None:
        self._pending_feedback.append(
            CommandFeedback(
                mission_id=command.mission_id,
                action_instance_id=command.action_instance_id,
                command_id=command.command_id,
                group_id=command.group_id,
                robot_id=command.robot_id,
                status=status,
                error_code=error_code,
                error_message=error_message,
                timestamp=self.clock.time(),
            )
        )

    def _mark_disconnected(self, detail: str) -> None:
        with self._lock:
            if self._connection_error is not None or self.closed:
                return
            self._connection_error = f"{self.group_id} TCP connection lost: {detail}"
        self._finish_active(
            CommandStatus.FAILURE,
            "TCP_DISCONNECTED",
            self._connection_error,
        )
