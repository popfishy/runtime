"""Unit tests for phase-2 domain model invariants."""

import pytest
from pydantic import ValidationError

from uav_bt_runtime import (
    CommandEnvelope,
    CommandFeedback,
    CommandStatus,
    CoordinationEvent,
    GroupRoster,
    MissionContext,
    RobotAssignment,
    TaskPlan,
    UavRuntimeState,
    UavSpec,
)


def make_spec(
    uav_id="A01",
    group_id="GroupA",
    role="Recon",
    is_leader=False,
    is_reserve=False,
):
    return UavSpec(
        uav_id=uav_id,
        group_id=group_id,
        role=role,
        is_leader=is_leader,
        is_reserve=is_reserve,
        capabilities={"MOVE_TO", "HOVER"},
    )


def make_plan(robot_id="A01", group_id="GroupA"):
    return TaskPlan(
        plan_id="prepare-a",
        group_id=group_id,
        robot_assignments={
            robot_id: RobotAssignment(
                robot_id=robot_id,
                command_type="MOVE_TO",
                payload={"target": {"x": 1.0, "y": 2.0, "z": 3.0}},
                terminal_action="HOVER",
            )
        },
    )


def make_context():
    specs = {
        "A01": make_spec("A01", is_leader=True),
        "A02": make_spec("A02", is_reserve=True),
    }
    states = {
        "A01": UavRuntimeState(uav_id="A01", online=True, available=True),
        "A02": UavRuntimeState(uav_id="A02", online=True, available=True),
    }
    plan = make_plan()
    return MissionContext(
        mission_id="mission-001",
        group_id="GroupA",
        uav_specs=specs,
        uav_states=states,
        roster=GroupRoster(
            group_id="GroupA",
            active_ids=["A01"],
            reserve_ids=["A02"],
        ),
        plans={plan.plan_id: plan},
    )


def test_uav_spec_is_immutable_and_rejects_leader_reserve():
    spec = make_spec(is_leader=True)
    with pytest.raises(ValidationError):
        spec.role = "Other"

    with pytest.raises(ValidationError, match="both leader and reserve"):
        make_spec(is_leader=True, is_reserve=True)


def test_uav_spec_rejects_unknown_fields_and_empty_capabilities():
    with pytest.raises(ValidationError, match="extra_forbidden"):
        UavSpec(
            uav_id="A01",
            group_id="GroupA",
            role="Recon",
            capabilities={"MOVE_TO"},
            unexpected=True,
        )

    with pytest.raises(ValidationError):
        UavSpec(uav_id="A01", group_id="GroupA", role="Recon", capabilities=set())


def test_runtime_state_rejects_inconsistent_availability():
    with pytest.raises(ValidationError, match="faulted UAV"):
        UavRuntimeState(uav_id="A01", faulted=True, available=True)

    with pytest.raises(ValidationError, match="current action"):
        UavRuntimeState(uav_id="A01", available=False, current_action="Coverage")


def test_roster_sets_are_disjoint_unique_and_reserve_order_is_stable():
    roster = GroupRoster(
        group_id="GroupA",
        active_ids=["A01"],
        reserve_ids=["A03", "A02"],
    )
    assert roster.next_reserve_id() == "A03"
    assert roster.all_ids == {"A01", "A02", "A03"}

    with pytest.raises(ValidationError, match="duplicates"):
        GroupRoster(group_id="GroupA", active_ids=["A01", "A01"])

    with pytest.raises(ValidationError, match="disjoint"):
        GroupRoster(group_id="GroupA", active_ids=["A01"], reserve_ids=["A01"])


def test_roster_accounts_for_inactive_members_without_selecting_them():
    roster = GroupRoster(
        group_id="GroupB", active_ids=["B01"], inactive_ids=["B07", "B08"]
    )
    assert roster.all_ids == {"B01", "B07", "B08"}
    with pytest.raises(ValidationError, match="disjoint"):
        GroupRoster(group_id="GroupB", active_ids=["B01"], inactive_ids=["B01"])


def test_mission_context_correlates_specs_states_roster_and_plans():
    context = make_context()
    assert context.uav_specs["A01"].uav_id == context.uav_states["A01"].uav_id
    assert context.next_reserve_id() == "A02"
    assert context.plans["prepare-a"].robot_assignments["A01"].payload["target"]["z"] == 3.0


def test_mission_context_rejects_missing_state_and_cross_group_spec():
    valid = make_context()
    with pytest.raises(ValidationError, match="one-to-one"):
        MissionContext(
            mission_id=valid.mission_id,
            group_id=valid.group_id,
            uav_specs=valid.uav_specs,
            uav_states={"A01": valid.uav_states["A01"]},
            roster=valid.roster,
            plans=valid.plans,
        )

    specs = dict(valid.uav_specs)
    specs["B01"] = make_spec("B01", group_id="GroupB")
    states = dict(valid.uav_states)
    states["B01"] = UavRuntimeState(uav_id="B01")
    with pytest.raises(ValidationError, match="another Group"):
        MissionContext(
            mission_id=valid.mission_id,
            group_id="GroupA",
            uav_specs=specs,
            uav_states=states,
            roster=GroupRoster(
                group_id="GroupA",
                active_ids=["A01", "B01"],
                reserve_ids=["A02"],
            ),
        )


def test_mission_context_rejects_bad_reserve_and_unknown_plan_assignment():
    specs = {
        "A01": make_spec("A01", is_leader=True),
        "A02": make_spec("A02", is_reserve=False),
    }
    states = {uav_id: UavRuntimeState(uav_id=uav_id) for uav_id in specs}
    with pytest.raises(ValidationError, match="not configured as Reserve"):
        MissionContext(
            mission_id="mission-001",
            group_id="GroupA",
            uav_specs=specs,
            uav_states=states,
            roster=GroupRoster(
                group_id="GroupA", active_ids=["A01"], reserve_ids=["A02"]
            ),
        )

    specs["A02"] = make_spec("A02", is_reserve=True)
    unknown_plan = make_plan(robot_id="A99")
    with pytest.raises(ValidationError, match="unknown or cross-Group"):
        MissionContext(
            mission_id="mission-001",
            group_id="GroupA",
            uav_specs=specs,
            uav_states=states,
            roster=GroupRoster(
                group_id="GroupA", active_ids=["A01"], reserve_ids=["A02"]
            ),
            plans={unknown_plan.plan_id: unknown_plan},
        )


def test_task_plan_requires_assignment_key_to_match_robot_id():
    with pytest.raises(ValidationError, match="assignment key"):
        TaskPlan(
            plan_id="bad-plan",
            group_id="GroupA",
            robot_assignments={
                "A01": RobotAssignment(robot_id="A02", command_type="MOVE_TO")
            },
        )


def test_command_and_feedback_models_are_strict_and_per_uav():
    command = CommandEnvelope(
        mission_id="mission-001",
        action_instance_id="prepare-001",
        command_id="cmd-001",
        group_id="GroupA",
        robot_id="A01",
        command_type="MOVE_TO",
        payload={"target": [1.0, 2.0, 3.0]},
        timeout_s=30.0,
        issued_at=100.0,
        expires_at=130.0,
        sequence_number=1,
    )
    assert command.command_type == "MOVE_TO"

    feedback = CommandFeedback(
        mission_id=command.mission_id,
        action_instance_id=command.action_instance_id,
        command_id=command.command_id,
        group_id=command.group_id,
        robot_id=command.robot_id,
        status=CommandStatus.RUNNING,
        timestamp=101.0,
    )
    assert feedback.status is CommandStatus.RUNNING

    invalid_command = command.model_dump()
    invalid_command["expires_at"] = 99.0
    with pytest.raises(ValidationError, match="expires_at"):
        CommandEnvelope.model_validate(invalid_command)

    with pytest.raises(ValidationError):
        CommandFeedback(
            mission_id="mission-001",
            action_instance_id="prepare-001",
            command_id="cmd-001",
            group_id="GroupA",
            robot_id="A01",
            status="UNKNOWN",
            timestamp=101.0,
        )


def test_coordination_event_is_independent_from_command_messages():
    event = CoordinationEvent(
        mission_id="mission-001",
        event_id="event-001",
        source_group="GroupA",
        event_type="RECON_COMPLETE",
        sequence_number=2,
        timestamp=200.0,
    )
    assert event.event_type == "RECON_COMPLETE"
