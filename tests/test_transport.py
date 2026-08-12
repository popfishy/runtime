"""Tests for command correlation and the two independent transport boundaries."""

from uav_bt_runtime.clock import ManualClock
from uav_bt_runtime.executor import EventStore
from uav_bt_runtime.models import (
    CommandEnvelope,
    CommandFeedback,
    CommandStatus,
    CoordinationEvent,
)
from uav_bt_runtime.tracking import BatchResult, CommandBatchTracker
from uav_bt_runtime.transport import (
    InMemoryCommandTransport,
    InMemoryCoordinationBus,
    InMemoryCoordinationTransport,
    ScriptRule,
    ScriptedBackend,
    ScriptedOutcome,
)


def command(command_id="cmd-1", group_id="GroupA", robot_id="A01"):
    return CommandEnvelope(
        mission_id="mission-1",
        action_instance_id="Tree:0",
        command_id=command_id,
        group_id=group_id,
        robot_id=robot_id,
        command_type="MOVE_TO",
        payload={"_plan_id": "prepare-a"},
        timeout_s=10.0,
        issued_at=100.0,
        expires_at=110.0,
        sequence_number=1,
    )


def test_in_memory_command_lifecycle_and_tracker_are_monotonic():
    clock = ManualClock(epoch=100.0)
    transport = InMemoryCommandTransport("GroupA", clock)
    item = command()
    tracker = CommandBatchTracker([item])
    transport.send(item)

    first = transport.poll_feedback()
    assert [feedback.status for feedback in first] == [
        CommandStatus.ACCEPTED,
        CommandStatus.RUNNING,
    ]
    assert all(tracker.apply(feedback) for feedback in first)
    assert tracker.result == BatchResult.RUNNING

    assert transport.poll_feedback() == []
    final = transport.poll_feedback()
    assert [feedback.status for feedback in final] == [CommandStatus.SUCCESS]
    assert tracker.apply(final[0])
    assert tracker.result == BatchResult.SUCCESS

    late_running = final[0].model_copy(update={"status": CommandStatus.RUNNING})
    assert not tracker.apply(late_running)
    assert tracker.result == BatchResult.SUCCESS


def test_scripted_failure_cancel_and_cross_group_protection():
    clock = ManualClock(epoch=100.0)
    backend = ScriptedBackend(
        rules=[
            ScriptRule(
                robot_id="A01",
                outcome=ScriptedOutcome(status=CommandStatus.FAILURE, delay_polls=0),
            )
        ]
    )
    transport = InMemoryCommandTransport("GroupA", clock, backend)
    item = command()
    transport.send(item)
    transport.poll_feedback()
    final = transport.poll_feedback()
    assert final[0].status == CommandStatus.FAILURE

    other = command("cmd-2", robot_id="A02")
    transport.send(other)
    transport.cancel(other.command_id)
    assert CommandStatus.CANCELLED in {
        feedback.status for feedback in transport.poll_feedback()
    }

    try:
        transport.send(command("cmd-b", group_id="GroupB", robot_id="B01"))
    except ValueError as exc:
        assert "cannot send" in str(exc)
    else:
        raise AssertionError("cross-Group command was accepted")


def test_in_memory_transport_enforces_command_expiry():
    clock = ManualClock(epoch=100.0)
    transport = InMemoryCommandTransport("GroupA", clock)
    item = command()
    transport.send(item)
    transport.poll_feedback()
    clock.advance(11.0)
    feedback = transport.poll_feedback()
    assert [item.status for item in feedback] == [CommandStatus.TIMEOUT]


def test_tracker_rejects_mismatched_and_unknown_feedback():
    item = command()
    tracker = CommandBatchTracker([item])
    unknown = CommandFeedback(
        mission_id=item.mission_id,
        action_instance_id=item.action_instance_id,
        command_id="unknown",
        group_id=item.group_id,
        robot_id=item.robot_id,
        status=CommandStatus.SUCCESS,
        timestamp=101.0,
    )
    assert not tracker.apply(unknown)
    mismatch = unknown.model_copy(
        update={"command_id": item.command_id, "robot_id": "A99"}
    )
    assert not tracker.apply(mismatch)
    assert tracker.result == BatchResult.RUNNING


def test_coordination_bus_delivers_peer_events_and_can_drop_selected_types():
    bus = InMemoryCoordinationBus(drop_event_types=["RECON_COMPLETE"])
    group_a = InMemoryCoordinationTransport("GroupA", bus)
    group_b = InMemoryCoordinationTransport("GroupB", bus)
    ready = CoordinationEvent(
        mission_id="mission-1",
        event_id="ready-1",
        source_group="GroupA",
        event_type="GROUP_READY",
        sequence_number=1,
        timestamp=100.0,
    )
    group_a.publish(ready)
    assert group_a.poll_events() == []
    assert group_b.poll_events() == [ready]

    recon = ready.model_copy(
        update={"event_id": "recon-1", "event_type": "RECON_COMPLETE"}
    )
    group_a.publish(recon)
    assert group_b.poll_events() == []


def test_event_store_rejects_duplicates_and_out_of_order_sequences():
    store = EventStore()
    first = CoordinationEvent(
        mission_id="mission-1",
        event_id="event-2",
        source_group="GroupA",
        event_type="GROUP_READY",
        sequence_number=2,
        timestamp=100.0,
    )
    assert store.add(first)
    assert not store.add(first)
    stale = first.model_copy(
        update={"event_id": "event-1", "sequence_number": 1, "event_type": "OLD"}
    )
    assert not store.add(stale)
    assert not store.has("OLD", "GroupA")
