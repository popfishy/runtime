"""Focused Group A semantic-task tests for the 12+3 aircraft scene."""

from pathlib import Path

import pytest

from uav_bt_runtime.bt import NodeStatus
from uav_bt_runtime.clock import ManualClock
from uav_bt_runtime.models import CommandStatus
from uav_bt_runtime.package import load_mission_package
from uav_bt_runtime.task_testing import run_single_task
from uav_bt_runtime.transport import (
    InMemoryCommandTransport,
    ScriptRule,
    ScriptedBackend,
    ScriptedOutcome,
)


PACKAGE = Path(__file__).resolve().parents[1] / "examples" / "joint_mission" / "group_a"
INITIAL_ACTIVE = tuple(f"A{index:02d}" for index in range(1, 13))
FAULT_IDS = tuple(f"A{index:02d}" for index in range(7, 13))
RECOVERED_ACTIVE = tuple(
    [f"A{index:02d}" for index in range(1, 7)] + ["A13", "A14", "A15"]
)


def execute(task, backend=None):
    package = load_mission_package(PACKAGE, require_reviewed=False)
    clock = ManualClock()
    transport = InMemoryCommandTransport("GroupA", clock, backend)
    result = run_single_task(package, task, transport, clock)
    return package, transport, result


@pytest.mark.parametrize(
    "task, expected",
    [
        ("prepare", [("MOVE_TO", 12, 0)]),
        ("coverage-segment-1", [("FOLLOW_ROUTE", 12, 1)]),
        ("fault-exit", [("FAULT_EXIT", 6, 6)]),
        ("recovery", [("MOVE_TO", 9, 0)]),
        ("coverage-segment-2", [("FOLLOW_ROUTE", 9, 1)]),
        ("hold-end", [("HOVER", 9, 0)]),
    ],
)
def test_group_a_task_command_batches(task, expected):
    _, _, result = execute(task)
    assert result.status == NodeStatus.SUCCESS
    assert [
        (goal.command_type, goal.robot_count, goal.route_count)
        for goal in result.goals
    ] == expected
    assert not result.failed_robot_ids
    assert not ({"TAKEOFF", "LAND"} & {goal.command_type for goal in result.goals})


def test_fault_exit_updates_six_members_and_routes_are_distinct():
    package, _, result = execute("fault-exit")
    assert result.roster_failed_ids == FAULT_IDS
    assert result.roster_active_ids == tuple(f"A{index:02d}" for index in range(1, 7))
    exits = []
    for robot_id in FAULT_IDS:
        state = package.context.uav_states[robot_id]
        assert state.faulted and not state.available
        route = package.context.plans["fault-exit"].robot_assignments[robot_id].payload[
            "waypoints"
        ]
        assert route[0]["z"] == pytest.approx(8.0)
        assert route[-1]["z"] == pytest.approx(8.0)
        assert 70.0 <= route[0]["y"] <= 80.0
        exits.append((route[-1]["x"], route[-1]["y"]))
    assert len(set(exits)) == 6


def test_recovery_activates_three_reserves_after_six_failures():
    _, _, result = execute("recovery")
    assert result.roster_active_ids == RECOVERED_ACTIVE
    assert result.roster_reserve_ids == ()
    assert result.roster_failed_ids == FAULT_IDS
    assert result.goals[0].robot_ids == RECOVERED_ACTIVE


def test_second_coverage_stores_only_leader_route_and_finishes_at_upper_hold():
    package, _, result = execute("coverage-segment-2")
    route_goal = result.goals[0]
    assert route_goal.robot_ids == RECOVERED_ACTIVE
    assert route_goal.route_count == 1
    assert route_goal.waypoint_count == 11
    plan = package.context.plans["coverage-segment-2"]
    final_y = plan.robot_assignments["A01"].payload["waypoints"][-1]["y"]
    assert 130.0 <= final_y <= 145.0
    assert all(
        assignment.payload == {"formation_follow": True}
        for robot_id, assignment in plan.robot_assignments.items()
        if robot_id != "A01"
    )


def test_recovery_failure_is_not_reported_as_success():
    backend = ScriptedBackend(
        rules=[
            ScriptRule(
                plan_id="recovery-a",
                robot_id="A15",
                command_type="MOVE_TO",
                outcome=ScriptedOutcome(
                    status=CommandStatus.FAILURE,
                    delay_polls=0,
                    error_code="TEST_RECOVERY_FAILURE",
                ),
            )
        ]
    )
    _, _, result = execute("recovery", backend)
    assert result.status == NodeStatus.FAILURE
    assert result.failed_robot_ids == ("A15",)
    assert [goal.command_type for goal in result.goals] == ["MOVE_TO"]


def test_task_timeout_cancels_the_active_batch():
    package = load_mission_package(PACKAGE, require_reviewed=False)
    clock = ManualClock()
    transport = InMemoryCommandTransport(
        "GroupA", clock, ScriptedBackend(default=ScriptedOutcome(delay_polls=100))
    )
    result = run_single_task(
        package, "fault-exit", transport, clock, max_ticks=2, tick_seconds=0.1
    )
    assert result.status == NodeStatus.TIMEOUT
    assert {item.status for item in transport.poll_feedback()} == {CommandStatus.CANCELLED}
