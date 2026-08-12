"""Focused execution helpers for one reviewed Group task at a time."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from .actions import build_action_registry
from .bt import ActionNode, NodeStatus
from .clock import Clock, ManualClock
from .executor import RuntimeServices
from .mission_log import MemoryMissionLogger, MissionLogger
from .models import (
    CommandEnvelope,
    CommandFeedback,
    CommandStatus,
    GroupRoster,
    MissionContext,
)
from .package import MissionPackage
from .transport import InMemoryCoordinationBus, InMemoryCoordinationTransport


@dataclass(frozen=True)
class TaskDefinition:
    group_id: str
    action_id: str
    plan_id: str
    timeout_s: float
    precondition: str = "initial"


@dataclass(frozen=True)
class TaskBatchSummary:
    command_type: str
    robot_ids: Tuple[str, ...]
    route_count: int
    waypoint_count: int
    strike_target_count: int

    @property
    def robot_count(self) -> int:
        return len(self.robot_ids)

    def to_dict(self) -> Dict[str, object]:
        return {
            "command_type": self.command_type,
            "robot_count": self.robot_count,
            "robot_ids": list(self.robot_ids),
            "route_count": self.route_count,
            "waypoint_count": self.waypoint_count,
            "strike_target_count": self.strike_target_count,
        }


@dataclass(frozen=True)
class TaskFailureDetail:
    robot_id: str
    status: CommandStatus
    error_code: Optional[str]
    error_message: Optional[str]

    def to_dict(self) -> Dict[str, object]:
        return {
            "robot_id": self.robot_id,
            "status": self.status.value,
            "error_code": self.error_code,
            "error_message": self.error_message,
        }


@dataclass(frozen=True)
class TaskTestResult:
    group_id: str
    task: str
    status: NodeStatus
    ticks: int
    goals: Tuple[TaskBatchSummary, ...]
    failed_robot_ids: Tuple[str, ...]
    failure_details: Tuple[TaskFailureDetail, ...]
    roster_active_ids: Tuple[str, ...]
    roster_reserve_ids: Tuple[str, ...]
    roster_failed_ids: Tuple[str, ...]
    roster_inactive_ids: Tuple[str, ...]

    def to_dict(self) -> Dict[str, object]:
        return {
            "group_id": self.group_id,
            "task": self.task,
            "status": self.status.value,
            "ticks": self.ticks,
            "goals": [goal.to_dict() for goal in self.goals],
            "failed_robot_ids": list(self.failed_robot_ids),
            "failure_details": [item.to_dict() for item in self.failure_details],
            "roster": {
                "active_ids": list(self.roster_active_ids),
                "reserve_ids": list(self.roster_reserve_ids),
                "failed_ids": list(self.roster_failed_ids),
                "inactive_ids": list(self.roster_inactive_ids),
            },
        }


TASK_DEFINITIONS: Dict[str, Dict[str, TaskDefinition]] = {
    "GroupA": {
        "prepare": TaskDefinition("GroupA", "PrepareGroupA", "prepare-a", 30.0),
        "coverage-segment-1": TaskDefinition(
            "GroupA", "CoverageSegment1", "coverage-segment-1", 60.0
        ),
        "fault-exit": TaskDefinition(
            "GroupA", "SimulateDamage", "fault-exit", 30.0
        ),
        "recovery": TaskDefinition(
            "GroupA", "RecoverReconGroup", "recovery-a", 60.0, "damaged"
        ),
        "coverage-segment-2": TaskDefinition(
            "GroupA", "CoverageSegment2", "coverage-segment-2", 60.0, "recovered"
        ),
        "hold-end": TaskDefinition(
            "GroupA", "HoldCoverageEnd", "hold-a-end", 30.0, "recovered"
        ),
    },
    "GroupB": {
        "strike-targets": TaskDefinition(
            "GroupB", "StrikeTargets", "strike-targets", 300.0
        ),
        "strike-target-b01-test": TaskDefinition(
            "GroupB", "StrikeTargets", "strike-target-b01-test", 300.0
        ),
        "return-strike-uavs": TaskDefinition(
            "GroupB", "ReturnStrikeUavs", "return-strike-uavs", 300.0
        ),
    },
}


class RecordingCommandTransport:
    """Record semantic command batches while preserving the real transport."""

    def __init__(self, delegate) -> None:
        self.delegate = delegate
        self.batches: List[Tuple[CommandEnvelope, ...]] = []
        self.feedback_history: List[CommandFeedback] = []

    def send(self, command: CommandEnvelope) -> None:
        self.batches.append((command,))
        self.delegate.send(command)

    def send_batch(self, commands: Sequence[CommandEnvelope]) -> None:
        batch = tuple(commands)
        if not batch:
            raise ValueError("recorded command batch cannot be empty")
        self.batches.append(batch)
        delegate_send_batch = getattr(self.delegate, "send_batch", None)
        if callable(delegate_send_batch):
            delegate_send_batch(batch)
            return
        for command in batch:
            self.delegate.send(command)

    def cancel(self, command_id: str) -> None:
        self.delegate.cancel(command_id)

    def poll_feedback(self) -> List[CommandFeedback]:
        feedback = self.delegate.poll_feedback()
        self.feedback_history.extend(feedback)
        return feedback

    def emergency_hold(self, mission_id: str) -> None:
        emergency_hold = getattr(self.delegate, "emergency_hold", None)
        if not callable(emergency_hold):
            raise RuntimeError("command transport does not provide EmergencyHold")
        emergency_hold(mission_id)

    def close(self) -> None:
        self.delegate.close()


def available_tasks(group_id: str) -> Tuple[str, ...]:
    if group_id not in TASK_DEFINITIONS:
        raise ValueError(f"unsupported Group {group_id!r}")
    return tuple(TASK_DEFINITIONS[group_id])


def _mark_damaged(context: MissionContext) -> None:
    fault_ids = list(context.plans["fault-exit"].robot_assignments)
    active_ids = [item for item in context.roster.active_ids if item not in fault_ids]
    for robot_id in fault_ids:
        state = context.uav_states[robot_id]
        context.uav_states[robot_id] = state.model_copy(
            update={"available": False, "faulted": True, "current_action": None}
        )
    context.roster = GroupRoster(
        group_id=context.group_id,
        active_ids=active_ids,
        reserve_ids=list(context.roster.reserve_ids),
        failed_ids=fault_ids,
        inactive_ids=list(context.roster.inactive_ids),
    )


def _mark_recovered(context: MissionContext) -> None:
    _mark_damaged(context)
    needed = int(context.planned_recovery_count)
    activated = list(context.roster.reserve_ids[:needed])
    context.roster = GroupRoster(
        group_id=context.group_id,
        active_ids=list(context.roster.active_ids) + activated,
        reserve_ids=list(context.roster.reserve_ids[needed:]),
        failed_ids=list(context.roster.failed_ids),
        inactive_ids=list(context.roster.inactive_ids),
    )


def apply_task_precondition(context: MissionContext, precondition: str) -> None:
    if precondition == "initial":
        return
    if context.group_id != "GroupA":
        raise ValueError("only GroupA task tests have roster preconditions")
    if precondition == "damaged":
        _mark_damaged(context)
        return
    if precondition == "recovered":
        _mark_recovered(context)
        return
    raise ValueError(f"unsupported task precondition {precondition!r}")


def _action_node(definition: TaskDefinition, suffix: str = "0") -> ActionNode:
    registration = build_action_registry()[definition.action_id]
    attributes = {
        "plan_id": definition.plan_id,
        "timeout_s": str(definition.timeout_s),
    }
    instance_id = f"TaskTest:{definition.action_id}:{suffix}"
    return ActionNode(
        instance_id,
        definition.action_id,
        registration.factory(instance_id, attributes),
        timeout_s=definition.timeout_s,
    )


def build_task_node(package: MissionPackage, task: str):
    group_id = package.context.group_id
    try:
        definition = TASK_DEFINITIONS[group_id][task]
    except KeyError as exc:
        raise ValueError(
            f"unsupported task {task!r} for {group_id}; "
            f"available={list(available_tasks(group_id))}"
        ) from exc
    apply_task_precondition(package.context, definition.precondition)
    return _action_node(definition)


def _summarize_batch(commands: Tuple[CommandEnvelope, ...]) -> TaskBatchSummary:
    command_types = {command.command_type for command in commands}
    if len(command_types) != 1:
        raise ValueError("one recorded task batch must contain one command type")
    route_count = 0
    waypoint_count = 0
    strike_target_count = 0
    for command in commands:
        waypoints = command.payload.get("waypoints")
        if command.command_type in {"FOLLOW_ROUTE", "FAULT_EXIT", "RETURN"} and isinstance(waypoints, list):
            route_count += 1
            waypoint_count += len(waypoints)
        if command.command_type == "ATTACK" and command.payload.get("target_id"):
            strike_target_count += 1
    return TaskBatchSummary(
        command_type=commands[0].command_type,
        robot_ids=tuple(command.robot_id for command in commands),
        route_count=route_count,
        waypoint_count=waypoint_count,
        strike_target_count=strike_target_count,
    )


def run_single_task(
    package: MissionPackage,
    task: str,
    command_transport,
    clock: Clock,
    max_ticks: int = 1000,
    tick_seconds: float = 0.1,
    wait_for_next_tick: Optional[Callable[[], None]] = None,
    logger: Optional[MissionLogger] = None,
) -> TaskTestResult:
    """Run one semantic task without executing preceding/following XML nodes."""

    if max_ticks <= 0 or tick_seconds <= 0:
        raise ValueError("max_ticks and tick_seconds must be positive")
    node = build_task_node(package, task)
    recording = RecordingCommandTransport(command_transport)
    bus = InMemoryCoordinationBus()
    services = RuntimeServices(
        mission=package.context,
        command_transport=recording,
        coordination_transport=InMemoryCoordinationTransport(
            package.context.group_id, bus
        ),
        clock=clock,
        logger=logger or MemoryMissionLogger(clock),
    )

    terminal = {
        NodeStatus.SUCCESS,
        NodeStatus.FAILURE,
        NodeStatus.CANCELLED,
        NodeStatus.TIMEOUT,
    }
    status = NodeStatus.IDLE
    ticks = 0
    while ticks < max_ticks and status not in terminal:
        status = node.tick(services)
        ticks += 1
        if status in terminal:
            break
        if wait_for_next_tick is not None:
            wait_for_next_tick()
        elif isinstance(clock, ManualClock):
            clock.advance(tick_seconds)
    if status not in terminal:
        node.halt(services, NodeStatus.TIMEOUT)
        status = NodeStatus.TIMEOUT

    failed_statuses = {
        CommandStatus.FAILURE,
        CommandStatus.REJECTED,
        CommandStatus.CANCELLED,
        CommandStatus.TIMEOUT,
    }
    failed_ids = sorted(
        {
            feedback.robot_id
            for feedback in recording.feedback_history
            if feedback.status in failed_statuses
        }
    )
    failure_details = tuple(
        TaskFailureDetail(
            robot_id=feedback.robot_id,
            status=feedback.status,
            error_code=feedback.error_code,
            error_message=feedback.error_message,
        )
        for feedback in recording.feedback_history
        if feedback.status in failed_statuses
    )
    roster = package.context.roster
    return TaskTestResult(
        group_id=package.context.group_id,
        task=task,
        status=status,
        ticks=ticks,
        goals=tuple(_summarize_batch(batch) for batch in recording.batches),
        failed_robot_ids=tuple(failed_ids),
        failure_details=failure_details,
        roster_active_ids=tuple(roster.active_ids),
        roster_reserve_ids=tuple(roster.reserve_ids),
        roster_failed_ids=tuple(roster.failed_ids),
        roster_inactive_ids=tuple(roster.inactive_ids),
    )
