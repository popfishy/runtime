"""Strict domain models shared by the mission runtime and future adapters."""

from __future__ import annotations

from enum import Enum
from typing import Dict, FrozenSet, List, Optional, Set

from typing_extensions import Annotated

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    JsonValue,
    NonNegativeFloat,
    NonNegativeInt,
    PositiveFloat,
    StringConstraints,
    field_validator,
    model_validator,
)


Identifier = Annotated[
    str,
    StringConstraints(
        strip_whitespace=True,
        min_length=1,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9_.:-]*$",
    ),
]


class StrictModel(BaseModel):
    """Base model for fail-closed external and runtime data."""

    model_config = ConfigDict(extra="forbid", validate_assignment=True)


class CommandStatus(str, Enum):
    """States returned by a lower execution layer."""

    ACCEPTED = "ACCEPTED"
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    REJECTED = "REJECTED"
    CANCELLED = "CANCELLED"
    TIMEOUT = "TIMEOUT"


class CompletionPolicy(str, Enum):
    """Phase-2 task plans require every assigned UAV to finish."""

    ALL = "ALL"


class UavSpec(StrictModel):
    """Immutable identity, group membership, and static UAV capability data."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    uav_id: Identifier
    group_id: Identifier
    role: Identifier
    is_leader: bool = False
    is_reserve: bool = False
    capabilities: FrozenSet[Identifier] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_leader_reserve_role(self) -> "UavSpec":
        if self.is_leader and self.is_reserve:
            raise ValueError("a UAV cannot be both leader and reserve")
        return self


class UavRuntimeState(StrictModel):
    """Mutable high-level state correlated with :class:`UavSpec` by uav_id."""

    uav_id: Identifier
    online: bool = False
    available: bool = False
    faulted: bool = False
    current_action: Optional[Identifier] = None
    last_feedback_time: Optional[NonNegativeFloat] = None

    @model_validator(mode="after")
    def validate_availability(self) -> "UavRuntimeState":
        if self.faulted and self.available:
            raise ValueError("a faulted UAV cannot be available")
        if self.current_action is not None and not self.available:
            raise ValueError("an unavailable UAV cannot have a current action")
        return self


class GroupRoster(StrictModel):
    """Current Group membership; reserve_ids order defines activation priority.

    ``inactive_ids`` accounts for registered aircraft that are intentionally not
    available to this mission (for example, Group B aircraft without outdoor
    positioning).  Runtime selectors must never dispatch commands to this set.
    """

    group_id: Identifier
    active_ids: List[Identifier] = Field(default_factory=list)
    reserve_ids: List[Identifier] = Field(default_factory=list)
    failed_ids: List[Identifier] = Field(default_factory=list)
    inactive_ids: List[Identifier] = Field(default_factory=list)

    @field_validator("active_ids", "reserve_ids", "failed_ids", "inactive_ids")
    @classmethod
    def reject_duplicate_ids(cls, value: List[str]) -> List[str]:
        if len(value) != len(set(value)):
            raise ValueError("a roster ID list cannot contain duplicates")
        return value

    @model_validator(mode="after")
    def validate_disjoint_membership(self) -> "GroupRoster":
        active = set(self.active_ids)
        reserve = set(self.reserve_ids)
        failed = set(self.failed_ids)
        inactive = set(self.inactive_ids)
        groups = (active, reserve, failed, inactive)
        if any(groups[left] & groups[right] for left in range(4) for right in range(left + 1, 4)):
            raise ValueError(
                "active, reserve, failed, and inactive roster IDs must be disjoint"
            )
        return self

    @property
    def all_ids(self) -> Set[str]:
        """Return every UAV currently accounted for by this Group."""

        return (
            set(self.active_ids)
            | set(self.reserve_ids)
            | set(self.failed_ids)
            | set(self.inactive_ids)
        )

    def next_reserve_id(self) -> Optional[str]:
        """Return the highest-priority reserve without mutating the roster."""

        return self.reserve_ids[0] if self.reserve_ids else None


class CommandEnvelope(StrictModel):
    """Canonical per-UAV command independent of a concrete transport."""

    schema_version: Identifier = "1.0"
    mission_id: Identifier
    action_instance_id: Identifier
    command_id: Identifier
    group_id: Identifier
    robot_id: Identifier
    command_type: Identifier
    payload: Dict[str, JsonValue] = Field(default_factory=dict)
    timeout_s: PositiveFloat
    issued_at: NonNegativeFloat
    expires_at: NonNegativeFloat
    sequence_number: NonNegativeInt

    @model_validator(mode="after")
    def validate_expiry(self) -> "CommandEnvelope":
        if self.expires_at <= self.issued_at:
            raise ValueError("expires_at must be later than issued_at")
        return self


class CommandFeedback(StrictModel):
    """Canonical per-UAV feedback correlated by command_id."""

    schema_version: Identifier = "1.0"
    mission_id: Identifier
    action_instance_id: Identifier
    command_id: Identifier
    group_id: Identifier
    robot_id: Identifier
    status: CommandStatus
    error_code: Optional[Identifier] = None
    error_message: Optional[str] = None
    timestamp: NonNegativeFloat


class CoordinationEvent(StrictModel):
    """A task-stage event exchanged between Group executors."""

    schema_version: Identifier = "1.0"
    mission_id: Identifier
    event_id: Identifier
    source_group: Identifier
    event_type: Identifier
    sequence_number: NonNegativeInt
    timestamp: NonNegativeFloat


class RobotAssignment(StrictModel):
    """One UAV's reviewed input for a task plan."""

    robot_id: Identifier
    command_type: Identifier
    payload: Dict[str, JsonValue] = Field(default_factory=dict)
    terminal_action: Optional[Identifier] = None


class TaskPlan(StrictModel):
    """A reviewed, Group-owned plan referenced later by an XML plan_id."""

    plan_id: Identifier
    group_id: Identifier
    robot_assignments: Dict[str, RobotAssignment] = Field(min_length=1)
    completion_policy: CompletionPolicy = CompletionPolicy.ALL

    @model_validator(mode="after")
    def validate_assignment_keys(self) -> "TaskPlan":
        for robot_id, assignment in self.robot_assignments.items():
            if robot_id != assignment.robot_id:
                raise ValueError(
                    "robot assignment key must match assignment.robot_id: "
                    f"{robot_id!r} != {assignment.robot_id!r}"
                )
        return self


class MissionContext(StrictModel):
    """Validated state container for one Group's future behavior-tree executor."""

    mission_id: Identifier
    group_id: Identifier
    uav_specs: Dict[str, UavSpec]
    uav_states: Dict[str, UavRuntimeState]
    roster: GroupRoster
    plans: Dict[str, TaskPlan] = Field(default_factory=dict)
    blackboard: Dict[str, JsonValue] = Field(default_factory=dict)
    current_phase: Optional[Identifier] = None
    planned_fault_count: NonNegativeInt = 0
    planned_recovery_count: NonNegativeInt = 0

    @model_validator(mode="after")
    def validate_context_consistency(self) -> "MissionContext":
        spec_ids = set(self.uav_specs)
        state_ids = set(self.uav_states)

        for uav_id, spec in self.uav_specs.items():
            if uav_id != spec.uav_id:
                raise ValueError(f"UAV spec key {uav_id!r} does not match spec.uav_id")
            if spec.group_id != self.group_id:
                raise ValueError(f"UAV {uav_id!r} belongs to another Group")

        for uav_id, state in self.uav_states.items():
            if uav_id != state.uav_id:
                raise ValueError(f"UAV state key {uav_id!r} does not match state.uav_id")

        if spec_ids != state_ids:
            missing_states = sorted(spec_ids - state_ids)
            extra_states = sorted(state_ids - spec_ids)
            raise ValueError(
                "UAV specs and states must have a one-to-one ID mapping; "
                f"missing_states={missing_states}, extra_states={extra_states}"
            )

        if self.roster.group_id != self.group_id:
            raise ValueError("roster belongs to another Group")
        if self.roster.all_ids != spec_ids:
            missing_roster = sorted(spec_ids - self.roster.all_ids)
            unknown_roster = sorted(self.roster.all_ids - spec_ids)
            raise ValueError(
                "roster must account for every Group UAV exactly once; "
                f"missing={missing_roster}, unknown={unknown_roster}"
            )

        for reserve_id in self.roster.reserve_ids:
            spec = self.uav_specs[reserve_id]
            if not spec.is_reserve:
                raise ValueError(f"roster reserve {reserve_id!r} is not configured as Reserve")
            if spec.is_leader:
                raise ValueError(f"Leader {reserve_id!r} cannot be a standby Reserve")

        for inactive_id in self.roster.inactive_ids:
            state = self.uav_states[inactive_id]
            if state.available or state.current_action is not None:
                raise ValueError(
                    f"inactive UAV {inactive_id!r} cannot be available or assigned"
                )

        if self.planned_recovery_count > self.planned_fault_count:
            raise ValueError(
                "planned_recovery_count cannot exceed planned_fault_count"
            )

        for plan_id, plan in self.plans.items():
            if plan_id != plan.plan_id:
                raise ValueError(f"plan key {plan_id!r} does not match plan.plan_id")
            if plan.group_id != self.group_id:
                raise ValueError(f"plan {plan_id!r} belongs to another Group")
            unknown_assignments = set(plan.robot_assignments) - spec_ids
            if unknown_assignments:
                raise ValueError(
                    f"plan {plan_id!r} assigns unknown or cross-Group UAVs: "
                    f"{sorted(unknown_assignments)}"
                )
            for robot_id, assignment in plan.robot_assignments.items():
                capabilities = self.uav_specs[robot_id].capabilities
                if assignment.command_type not in capabilities:
                    raise ValueError(
                        f"UAV {robot_id!r} lacks capability {assignment.command_type!r} "
                        f"required by plan {plan_id!r}"
                    )
                if (
                    assignment.terminal_action is not None
                    and assignment.terminal_action not in capabilities
                ):
                    raise ValueError(
                        f"UAV {robot_id!r} lacks terminal capability "
                        f"{assignment.terminal_action!r} required by plan {plan_id!r}"
                    )

        return self

    def next_reserve_id(self) -> Optional[str]:
        """Return the deterministic next reserve ID for future action handlers."""

        return self.roster.next_reserve_id()
