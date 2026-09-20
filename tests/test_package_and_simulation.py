"""Mission-package, CLI, and full 31-UAV joint-flow tests."""

import hashlib
import json
import shutil
from pathlib import Path

import pytest

from uav_bt_runtime.cli import main
from uav_bt_runtime.package import (
    MissionPackageError,
    load_mission_package,
    validate_joint_packages,
)
from uav_bt_runtime.simulation import run_joint_simulation


EXAMPLES = Path(__file__).resolve().parents[1] / "examples" / "joint_mission"


def packages():
    return (
        load_mission_package(EXAMPLES / "group_a", require_reviewed=False),
        load_mission_package(EXAMPLES / "group_b", require_reviewed=False),
    )


def update_hash(package_dir, relative_path):
    manifest_path = package_dir / "mission.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["file_hashes"][relative_path] = hashlib.sha256(
        (package_dir / relative_path).read_bytes()
    ).hexdigest()
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def test_candidate_packages_are_jointly_valid_and_have_31_uavs():
    group_a, group_b = packages()
    validate_joint_packages(group_a, group_b)
    assert len(group_a.context.uav_specs) == 15
    assert len(group_b.context.uav_specs) == 16
    assert group_a.manifest.reviewed is False
    assert group_b.manifest.reviewed is False
    assert group_a.world.flight_area.max_x == 100.0
    assert group_a.world.flight_area.max_y == 150.0
    assert set(group_a.world.targets) == {"target-1", "target-2"}


def test_plans_match_the_agreed_physical_scene():
    group_a, group_b = packages()
    plans_a = group_a.context.plans
    assert set(plans_a["fault-exit"].robot_assignments) == {
        "A07", "A08", "A09", "A10", "A11", "A12"
    }
    assert set(plans_a["prepare-a"].robot_assignments) == {
        f"A{index:02d}" for index in range(1, 13)
    }
    assert set(plans_a["recovery-a"].robot_assignments) == {
        "A01", "A02", "A03", "A04", "A05", "A06", "A13", "A14", "A15"
    }
    for plan_id in ("coverage-segment-1", "coverage-segment-2"):
        plan = plans_a[plan_id]
        route_owners = [
            robot_id
            for robot_id, item in plan.robot_assignments.items()
            if item.payload.get("waypoints")
        ]
        assert route_owners == ["A01"]
    segment1 = plans_a["coverage-segment-1"].robot_assignments["A01"].payload[
        "waypoints"
    ]
    segment2 = plans_a["coverage-segment-2"].robot_assignments["A01"].payload[
        "waypoints"
    ]
    assert segment1[-1] == segment2[0]
    assert segment1[-1]["y"] == 75.0
    assert max(point["y"] for point in segment1[:-1]) < 75.0
    assert min(point["y"] for point in segment2[1:]) > 75.0
    assert segment1[-2]["y"] > segment1[0]["y"]
    assert segment2[-1]["y"] > segment2[1]["y"]
    assert 130.0 <= segment2[-1]["y"] <= 145.0

    assert set(group_b.context.roster.active_ids) == {
        f"B{index:02d}" for index in range(1, 7)
    }
    assert set(group_b.context.roster.inactive_ids) == {
        f"B{index:02d}" for index in range(7, 17)
    }
    assert set(group_b.context.plans["strike-targets"].robot_assignments) == {
        "B01", "B02"
    }
    assert set(group_b.context.plans["return-strike-uavs"].robot_assignments) == {
        "B01", "B02"
    }


def test_package_rejects_unreviewed_tampered_and_unknown_xml(tmp_path):
    copied = tmp_path / "group_a"
    shutil.copytree(EXAMPLES / "group_a", copied)
    with pytest.raises(MissionPackageError, match="not marked as reviewed"):
        load_mission_package(copied)

    manifest_path = copied / "mission.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["reviewed"] = True
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    tree_path = copied / "tree.xml"
    tree_path.write_text(tree_path.read_text(encoding="utf-8") + "<!--tamper-->\n", encoding="utf-8")
    with pytest.raises(MissionPackageError, match="SHA-256 mismatch"):
        load_mission_package(copied)

    tree_path.write_text(
        '<root main_tree_to_execute="Bad"><BehaviorTree ID="Bad">'
        '<Action ID="Unknown"/></BehaviorTree></root>\n',
        encoding="utf-8",
    )
    update_hash(copied, "tree.xml")
    with pytest.raises(MissionPackageError, match="unknown Action"):
        load_mission_package(copied)


def test_package_rejects_unknown_target_reference(tmp_path):
    copied = tmp_path / "group_b"
    shutil.copytree(EXAMPLES / "group_b", copied)
    plans_path = copied / "plans" / "plans.json"
    plans = json.loads(plans_path.read_text(encoding="utf-8"))
    strike = next(plan for plan in plans["plans"] if plan["plan_id"] == "strike-targets")
    strike["robot_assignments"]["B01"]["payload"]["target_id"] = "missing-target"
    plans_path.write_text(json.dumps(plans, indent=2) + "\n", encoding="utf-8")
    update_hash(copied, "plans/plans.json")
    with pytest.raises(MissionPackageError, match="unknown target_id"):
        load_mission_package(copied, require_reviewed=False)


def test_package_allows_route_coordinates_outside_configured_boundary(tmp_path):
    copied = tmp_path / "group_a"
    shutil.copytree(EXAMPLES / "group_a", copied)
    plans_path = copied / "plans" / "plans.json"
    plans = json.loads(plans_path.read_text(encoding="utf-8"))
    coverage = next(
        plan for plan in plans["plans"] if plan["plan_id"] == "coverage-segment-1"
    )
    coverage["robot_assignments"]["A01"]["payload"]["waypoints"][0]["x"] = 96.0
    plans_path.write_text(json.dumps(plans, indent=2) + "\n", encoding="utf-8")
    update_hash(copied, "plans/plans.json")
    package = load_mission_package(copied, require_reviewed=False)

    assert (
        package.context.plans["coverage-segment-1"]
        .robot_assignments["A01"]
        .payload["waypoints"][0]["x"]
        == 96.0
    )


def test_package_allows_target_outside_configured_boundary(tmp_path):
    copied = tmp_path / "group_b"
    shutil.copytree(EXAMPLES / "group_b", copied)
    world_path = copied / "world.json"
    world = json.loads(world_path.read_text(encoding="utf-8"))
    world["targets"]["target-2"].update({"x": 2.0, "y": 2.0})
    world_path.write_text(json.dumps(world, indent=2) + "\n", encoding="utf-8")
    update_hash(copied, "world.json")

    package = load_mission_package(copied, require_reviewed=False)

    assert package.world.targets["target-2"]["x"] == 2.0


@pytest.mark.parametrize(
    "scenario, expected_success",
    [
        ("normal", True),
        ("strike-failure", False),
        ("recovery-failure", False),
        ("partial-damage-failure", False),
        ("coordination-loss", False),
        ("return-failure", False),
    ],
)
def test_joint_simulation_scenarios(scenario, expected_success):
    group_a, group_b = packages()
    result = run_joint_simulation(group_a, group_b, scenario=scenario)
    assert result.success is expected_success
    if not expected_success:
        assert result.group_a_status.value == "FAILURE"
        assert result.group_b_status.value == "FAILURE"


def test_normal_simulation_logs_commands_without_takeoff_or_land(tmp_path):
    group_a, group_b = packages()
    result = run_joint_simulation(group_a, group_b, log_directory=tmp_path)
    assert result.success
    records = []
    for path in [tmp_path / "GroupA.jsonl", tmp_path / "GroupB.jsonl"]:
        records.extend(json.loads(line) for line in path.read_text(encoding="utf-8").splitlines())
    command_types = {
        record["command_type"]
        for record in records
        if record["record_type"] == "command_sent"
    }
    assert not ({"TAKEOFF", "LAND"} & command_types)
    assert {"MOVE_TO", "FOLLOW_ROUTE", "FAULT_EXIT", "ATTACK", "RETURN"} <= command_types


def test_partial_damage_failure_excludes_exited_uavs_from_safe_hold(tmp_path):
    group_a, group_b = packages()
    result = run_joint_simulation(
        group_a, group_b, scenario="partial-damage-failure", log_directory=tmp_path
    )
    assert not result.success
    records = [
        json.loads(line)
        for line in (tmp_path / "GroupA.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    safe_hold_ids = {
        record["robot_id"]
        for record in records
        if record.get("command_type") == "SAFE_HOLD"
    }
    assert "A07" not in safe_hold_ids
    assert "A12" in safe_hold_ids


def test_cli_validate_and_simulate_candidate_joint(capsys):
    assert main([
        "validate", str(EXAMPLES / "group_a"), str(EXAMPLES / "group_b"),
        "--allow-unreviewed",
    ]) == 0
    assert '"status": "VALID"' in capsys.readouterr().out
    assert main([
        "simulate-joint", "--group-a", str(EXAMPLES / "group_a"),
        "--group-b", str(EXAMPLES / "group_b"), "--scenario", "normal",
        "--allow-unreviewed",
    ]) == 0
    assert '"success": true' in capsys.readouterr().out


def test_cli_run_one_group_with_preconfirmed_peer_events(capsys):
    result = main([
        "run", "--package", str(EXAMPLES / "group_a"),
        "--peer-event", "GROUP_READY", "--peer-event", "STRIKE_COMPLETE",
        "--allow-unreviewed",
    ])
    assert result == 0
    assert '"status": "SUCCESS"' in capsys.readouterr().out
