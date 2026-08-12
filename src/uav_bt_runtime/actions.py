"""Semantic mission actions mapped from strict behavior-tree XML nodes."""

from __future__ import annotations

import uuid
from typing import Dict, List, Mapping, Optional, Sequence

from .bt import ActionRegistration, NodeStatus
from .executor import RuntimeServices
from .models import (
    CommandEnvelope,
    CommandFeedback,
    CommandStatus,
    GroupRoster,
    RobotAssignment,
    TaskPlan,
)
from .tracking import BatchResult, CommandBatchTracker


class PlanFanOutHandler:
    """Dispatch one reviewed per-UAV plan and aggregate all required results."""

    def __init__(self, instance_id: str, plan_id: str, command_timeout_s: float) -> None:
        self.instance_id = instance_id
        self.plan_id = plan_id
        self.command_timeout_s = command_timeout_s
        self.plan: Optional[TaskPlan] = None
        self.selected_ids: List[str] = []
        self.assignments: Dict[str, RobotAssignment] = {}
        self.tracker: Optional[CommandBatchTracker] = None
        self.stage = "PRIMARY"
        self.finished = False

    def select_ids(self, services: RuntimeServices, plan: TaskPlan) -> Sequence[str]:
        return list(plan.robot_assignments)

    def start(self, services: RuntimeServices) -> NodeStatus:
        plan = services.mission.plans.get(self.plan_id)
        if plan is None:
            raise ValueError(f"unknown plan_id {self.plan_id!r}")
        self.plan = plan
        self.selected_ids = list(self.select_ids(services, plan))
        if not self.selected_ids:
            raise ValueError(f"plan {self.plan_id!r} selected no UAVs")

        missing = set(self.selected_ids) - set(plan.robot_assignments)
        if missing:
            raise ValueError(
                f"plan {self.plan_id!r} has no assignments for {sorted(missing)}"
            )
        self.assignments = {
            uav_id: plan.robot_assignments[uav_id] for uav_id in self.selected_ids
        }
        for uav_id in self.selected_ids:
            state = services.mission.uav_states[uav_id]
            if not state.online or not state.available or state.faulted:
                raise ValueError(f"UAV {uav_id!r} is not available for {self.plan_id!r}")
            services.mission.uav_states[uav_id] = state.model_copy(
                update={"current_action": self.instance_id}
            )

        self._dispatch_primary(services)
        return NodeStatus.RUNNING

    def _make_command(
        self,
        services: RuntimeServices,
        uav_id: str,
        command_type: str,
        payload: Mapping[str, object],
        stage: str,
    ) -> CommandEnvelope:
        issued_at = services.clock.time()
        command_payload = dict(payload)
        command_payload["_plan_id"] = self.plan_id
        command_payload["_stage"] = stage
        return CommandEnvelope(
            mission_id=services.mission.mission_id,
            action_instance_id=self.instance_id,
            command_id=f"cmd-{uuid.uuid4().hex}",
            group_id=services.mission.group_id,
            robot_id=uav_id,
            command_type=command_type,
            payload=command_payload,
            timeout_s=self.command_timeout_s,
            issued_at=issued_at,
            expires_at=issued_at + self.command_timeout_s,
            sequence_number=services.next_sequence(),
        )

    def _send_batch(
        self, services: RuntimeServices, commands: List[CommandEnvelope]
    ) -> None:
        if not commands:
            raise ValueError(f"action {self.instance_id!r} produced an empty command batch")
        self.tracker = CommandBatchTracker(commands)
        send_batch = getattr(services.command_transport, "send_batch", None)
        if callable(send_batch):
            send_batch(commands)
        else:
            for command in commands:
                services.command_transport.send(command)
        for command in commands:
            services.logger.emit(
                "command_sent",
                mission_id=command.mission_id,
                group_id=command.group_id,
                action_instance_id=command.action_instance_id,
                command_id=command.command_id,
                robot_id=command.robot_id,
                command_type=command.command_type,
                plan_id=self.plan_id,
                stage=self.stage,
            )

    def _dispatch_primary(self, services: RuntimeServices) -> None:
        self.stage = "PRIMARY"
        commands = [
            self._make_command(
                services,
                uav_id,
                assignment.command_type,
                assignment.payload,
                self.stage,
            )
            for uav_id, assignment in self.assignments.items()
        ]
        self._send_batch(services, commands)

    def _dispatch_terminal(self, services: RuntimeServices) -> bool:
        terminal_assignments = {
            uav_id: assignment
            for uav_id, assignment in self.assignments.items()
            if assignment.terminal_action is not None
        }
        if not terminal_assignments:
            return False
        self.stage = "TERMINAL"
        commands = [
            self._make_command(
                services,
                uav_id,
                assignment.terminal_action or "HOVER",
                {},
                self.stage,
            )
            for uav_id, assignment in terminal_assignments.items()
        ]
        self._send_batch(services, commands)
        return True

    def _apply_feedback(self, services: RuntimeServices, feedback: CommandFeedback) -> None:
        if self.tracker is None or not self.tracker.apply(feedback):
            return
        state = services.mission.uav_states.get(feedback.robot_id)
        if state is not None:
            services.mission.uav_states[feedback.robot_id] = state.model_copy(
                update={"last_feedback_time": feedback.timestamp}
            )
        services.logger.emit(
            "command_feedback",
            mission_id=feedback.mission_id,
            group_id=feedback.group_id,
            action_instance_id=feedback.action_instance_id,
            command_id=feedback.command_id,
            robot_id=feedback.robot_id,
            status=feedback.status.value,
            error_code=feedback.error_code,
        )

    def tick(self, services: RuntimeServices) -> NodeStatus:
        if self.finished:
            return NodeStatus.SUCCESS
        if self.tracker is None:
            return NodeStatus.FAILURE

        for feedback in services.command_transport.poll_feedback():
            self._apply_feedback(services, feedback)

        if self.tracker.result == BatchResult.FAILURE:
            self._cancel_active(services)
            self._clear_current_action(services)
            self.on_failure(services)
            return NodeStatus.FAILURE

        if self.tracker.result == BatchResult.SUCCESS:
            if self.stage == "PRIMARY" and self._dispatch_terminal(services):
                return NodeStatus.RUNNING
            self._clear_current_action(services)
            self.on_success(services)
            self.finished = True
            return NodeStatus.SUCCESS
        return NodeStatus.RUNNING

    def _cancel_active(self, services: RuntimeServices) -> None:
        if self.tracker is None:
            return
        for command_id in self.tracker.active_command_ids:
            services.command_transport.cancel(command_id)

    def _clear_current_action(self, services: RuntimeServices) -> None:
        for uav_id in self.selected_ids:
            state = services.mission.uav_states[uav_id]
            if state.current_action == self.instance_id:
                services.mission.uav_states[uav_id] = state.model_copy(
                    update={"current_action": None}
                )

    def on_success(self, services: RuntimeServices) -> None:
        del services

    def on_failure(self, services: RuntimeServices) -> None:
        del services

    def cancel(self, services: RuntimeServices) -> None:
        self._cancel_active(services)
        self._clear_current_action(services)


class ActivePlanHandler(PlanFanOutHandler):
    """Require one assignment for every currently active UAV."""

    def select_ids(self, services: RuntimeServices, plan: TaskPlan) -> Sequence[str]:
        del plan
        return list(services.mission.roster.active_ids)


class SimulateDamageHandler(PlanFanOutHandler):
    """Mark configured non-Leader fault candidates unavailable after success."""

    def on_success(self, services: RuntimeServices) -> None:
        self._mark_exited(services, self.selected_ids)

    def on_failure(self, services: RuntimeServices) -> None:
        if self.tracker is None:
            return
        succeeded = [
            self.tracker.commands[command_id].robot_id
            for command_id, status in self.tracker.statuses.items()
            if status == CommandStatus.SUCCESS
        ]
        self._mark_exited(services, succeeded)

    @staticmethod
    def _mark_exited(services: RuntimeServices, exited_ids: Sequence[str]) -> None:
        mission = services.mission
        failed = list(mission.roster.failed_ids)
        active = list(mission.roster.active_ids)
        for uav_id in exited_ids:
            spec = mission.uav_specs[uav_id]
            if spec.is_leader:
                raise ValueError("simulated damage cannot target a Leader")
            if uav_id not in active:
                raise ValueError(f"fault candidate {uav_id!r} is not active")
            active.remove(uav_id)
            if uav_id not in failed:
                failed.append(uav_id)
            state = mission.uav_states[uav_id]
            mission.uav_states[uav_id] = state.model_copy(
                update={"current_action": None, "available": False, "faulted": True}
            )
        mission.roster = GroupRoster(
            group_id=mission.group_id,
            active_ids=active,
            reserve_ids=list(mission.roster.reserve_ids),
            failed_ids=failed,
            inactive_ids=list(mission.roster.inactive_ids),
        )


class RecoverReconGroupHandler(PlanFanOutHandler):
    """Activate the highest-priority fixed Reserve slots and restore membership."""

    def __init__(self, instance_id: str, plan_id: str, command_timeout_s: float) -> None:
        super().__init__(instance_id, plan_id, command_timeout_s)
        self.activated_reserves: List[str] = []

    def select_ids(self, services: RuntimeServices, plan: TaskPlan) -> Sequence[str]:
        del plan
        roster = services.mission.roster
        needed = int(services.mission.planned_recovery_count)
        if needed == 0:
            raise ValueError("recovery requested with planned_recovery_count=0")
        if len(roster.failed_ids) < needed:
            raise ValueError("fewer failed UAVs than the planned recovery count")
        if len(roster.reserve_ids) < needed:
            raise ValueError("not enough Reserve UAVs for fixed recovery")
        self.activated_reserves = list(roster.reserve_ids[:needed])
        return list(roster.active_ids) + self.activated_reserves

    def on_success(self, services: RuntimeServices) -> None:
        mission = services.mission
        remaining_reserves = [
            uav_id
            for uav_id in mission.roster.reserve_ids
            if uav_id not in self.activated_reserves
        ]
        mission.roster = GroupRoster(
            group_id=mission.group_id,
            active_ids=list(mission.roster.active_ids) + self.activated_reserves,
            reserve_ids=remaining_reserves,
            failed_ids=list(mission.roster.failed_ids),
            inactive_ids=list(mission.roster.inactive_ids),
        )


class PublishEventHandler:
    def __init__(self, event_type: str) -> None:
        self.event_type = event_type
        self.published = False

    def start(self, services: RuntimeServices) -> NodeStatus:
        if not self.published:
            services.publish_event(self.event_type)
            self.published = True
        return NodeStatus.SUCCESS

    def tick(self, services: RuntimeServices) -> NodeStatus:
        del services
        return NodeStatus.SUCCESS

    def cancel(self, services: RuntimeServices) -> None:
        del services


class WaitEventHandler:
    def __init__(self, event_type: str, source_group: str) -> None:
        self.event_type = event_type
        self.source_group = source_group

    def start(self, services: RuntimeServices) -> NodeStatus:
        return self.tick(services)

    def tick(self, services: RuntimeServices) -> NodeStatus:
        if services.events.has(self.event_type, self.source_group):
            return NodeStatus.SUCCESS
        return NodeStatus.RUNNING

    def cancel(self, services: RuntimeServices) -> None:
        del services


def _plan_registration(handler_type):
    def factory(instance_id: str, attributes: Mapping[str, str]):
        return handler_type(
            instance_id,
            attributes["plan_id"],
            float(attributes["timeout_s"]),
        )

    return ActionRegistration(
        factory=factory,
        required_attributes=frozenset({"plan_id", "timeout_s"}),
    )


def build_action_registry() -> Dict[str, ActionRegistration]:
    """Return the fixed v1 semantic Action whitelist."""

    def publish(event_type: str) -> ActionRegistration:
        return ActionRegistration(
            factory=lambda instance_id, attributes: PublishEventHandler(event_type)
        )

    def wait(event_type: str, source_group: str) -> ActionRegistration:
        return ActionRegistration(
            factory=lambda instance_id, attributes: WaitEventHandler(
                event_type, source_group
            ),
            required_attributes=frozenset({"timeout_s"}),
        )

    return {
        "PrepareGroupA": _plan_registration(PlanFanOutHandler),
        "PublishGroupReady": publish("GROUP_READY"),
        "WaitGroupAReady": wait("GROUP_READY", "GroupA"),
        "WaitGroupBReady": wait("GROUP_READY", "GroupB"),
        "CoverageSegment1": _plan_registration(ActivePlanHandler),
        "SimulateDamage": _plan_registration(SimulateDamageHandler),
        "RecoverReconGroup": _plan_registration(RecoverReconGroupHandler),
        "CoverageSegment2": _plan_registration(ActivePlanHandler),
        "PublishReconComplete": publish("RECON_COMPLETE"),
        "WaitReconComplete": wait("RECON_COMPLETE", "GroupA"),
        "StrikeTargets": _plan_registration(PlanFanOutHandler),
        "ReturnStrikeUavs": _plan_registration(PlanFanOutHandler),
        "PublishStrikeComplete": publish("STRIKE_COMPLETE"),
        "WaitStrikeComplete": wait("STRIKE_COMPLETE", "GroupB"),
        "HoldCoverageEnd": _plan_registration(ActivePlanHandler),
    }
