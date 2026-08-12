"""Focused Group B semantic-task tests for two targets and two aircraft."""

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


PACKAGE = Path(__file__).resolve().parents[1] / "examples" / "joint_mission" / "group_b"
STRIKE_IDS = ("B01", "B02")


def execute(task, backend=None):
    package = load_mission_package(PACKAGE, require_reviewed=False)
    clock = ManualClock()
    transport = InMemoryCommandTransport("GroupB", clock, backend)
    result = run_single_task(package, task, transport, clock)
    return package, transport, result


@pytest.mark.parametrize(
    "task, expected",
    [
        ("strike-targets", [("ATTACK", STRIKE_IDS)]),
        ("strike-target-b01-test", [("ATTACK", ("B01",))]),
        (
            "return-strike-uavs",
            [("RETURN", STRIKE_IDS)],
        ),
    ],
)
def test_group_b_task_command_batches(task, expected):
    _, _, result = execute(task)
    assert result.status == NodeStatus.SUCCESS
    assert [(goal.command_type, goal.robot_ids) for goal in result.goals] == expected
    assert not result.failed_robot_ids
    assert not ({"TAKEOFF", "LAND"} & {goal.command_type for goal in result.goals})


def test_two_strike_uavs_reference_two_known_targets():
    package, _, result = execute("strike-targets")
    assert result.goals[0].strike_target_count == 2
    assignments = package.context.plans["strike-targets"].robot_assignments
    assert assignments["B01"].payload["target_id"] == "target-1"
    assert assignments["B02"].payload["target_id"] == "target-2"
    for target_id in ("target-1", "target-2"):
        target = package.world.targets[target_id]
        assert isinstance(target["x"], (int, float))
        assert isinstance(target["y"], (int, float))
        assert isinstance(target["z"], (int, float))


def test_inactive_ground_nodes_are_never_selected():
    package, _, strike = execute("strike-targets")
    _, _, returned = execute("return-strike-uavs")
    inactive = set(package.context.roster.inactive_ids)
    assert inactive == {f"B{index:02d}" for index in range(7, 17)}
    assert all(not (inactive & set(goal.robot_ids)) for goal in strike.goals + returned.goals)


def test_one_strike_failure_fails_the_batch():
    backend = ScriptedBackend(
        rules=[
            ScriptRule(
                plan_id="strike-targets",
                robot_id="B02",
                outcome=ScriptedOutcome(
                    status=CommandStatus.FAILURE, error_code="TEST_STRIKE_FAILURE"
                ),
            )
        ]
    )
    _, _, result = execute("strike-targets", backend)
    assert result.status == NodeStatus.FAILURE
    assert result.failed_robot_ids == ("B02",)
    assert result.failure_details[0].error_code == "TEST_STRIKE_FAILURE"


def test_transport_rejects_cross_group_task_command():
    package = load_mission_package(PACKAGE, require_reviewed=False)
    clock = ManualClock()
    wrong_transport = InMemoryCommandTransport("GroupA", clock)
    result = run_single_task(package, "strike-targets", wrong_transport, clock)
    assert result.status == NodeStatus.FAILURE
    assert not result.goals or result.goals[0].robot_ids == STRIKE_IDS
