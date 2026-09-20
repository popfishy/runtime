"""Tests for strict, fail-closed JSON configuration loading."""

import json

import pytest

from uav_bt_runtime import (
    ConfigurationLoadError,
    PlanCatalog,
    UavCatalog,
    load_json_model,
    load_mission_context,
)


def write_json(path, data):
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def valid_files(tmp_path):
    robots = {
        "schema_version": "1.0",
        "uavs": [
            {
                "uav_id": "A01",
                "group_id": "GroupA",
                "role": "Recon",
                "is_leader": True,
                "is_reserve": False,
                "capabilities": ["MOVE_TO", "HOVER"],
            },
            {
                "uav_id": "A02",
                "group_id": "GroupA",
                "role": "ReconReserve",
                "is_leader": False,
                "is_reserve": True,
                "capabilities": ["MOVE_TO", "HOVER"],
            },
            {
                "uav_id": "B01",
                "group_id": "GroupB",
                "role": "Strike",
                "is_leader": True,
                "is_reserve": False,
                "capabilities": ["ATTACK", "RETURN"],
            },
        ],
    }
    mission = {
        "schema_version": "1.0",
        "mission_id": "mission-001",
        "group_id": "GroupA",
        "roster": {
            "group_id": "GroupA",
            "active_ids": ["A01"],
            "reserve_ids": ["A02"],
            "failed_ids": [],
        },
        "blackboard": {"armed": True},
        "planned_fault_count": 1,
        "planned_recovery_count": 1,
    }
    plans = {
        "schema_version": "1.0",
        "plans": [
            {
                "plan_id": "prepare-a",
                "group_id": "GroupA",
                "robot_assignments": {
                    "A01": {
                        "robot_id": "A01",
                        "command_type": "MOVE_TO",
                        "payload": {"target": [1.0, 2.0, 3.0]},
                        "terminal_action": "HOVER",
                    }
                },
                "completion_policy": "ALL",
            },
            {
                "plan_id": "strike-b",
                "group_id": "GroupB",
                "robot_assignments": {
                    "B01": {
                        "robot_id": "B01",
                        "command_type": "ATTACK",
                        "payload": {"target_id": "target-1"},
                    }
                },
            },
        ],
    }
    return (
        write_json(tmp_path / "robots.json", robots),
        write_json(tmp_path / "mission.json", mission),
        write_json(tmp_path / "plans.json", plans),
    )


def test_load_mission_context_filters_group_and_builds_one_to_one_state(tmp_path):
    robots_path, mission_path, plans_path = valid_files(tmp_path)
    context = load_mission_context(robots_path, mission_path, plans_path)

    assert set(context.uav_specs) == {"A01", "A02"}
    assert set(context.uav_states) == set(context.uav_specs)
    assert all(not state.online for state in context.uav_states.values())
    assert set(context.plans) == {"prepare-a"}
    assert context.next_reserve_id() == "A02"
    assert context.blackboard == {"armed": True}


def test_loader_rejects_invalid_json_and_unknown_fields(tmp_path):
    invalid = tmp_path / "invalid.json"
    invalid.write_text('{"uavs": [}', encoding="utf-8")
    with pytest.raises(ConfigurationLoadError, match="line 1"):
        load_json_model(invalid, UavCatalog)

    unknown = write_json(
        tmp_path / "unknown.json",
        {
            "schema_version": "1.0",
            "uavs": [],
            "unexpected": True,
        },
    )
    with pytest.raises(ConfigurationLoadError, match="extra_forbidden"):
        load_json_model(unknown, UavCatalog)


def test_loader_rejects_duplicate_uav_and_plan_ids(tmp_path):
    duplicate_uavs = write_json(
        tmp_path / "duplicate-uavs.json",
        {
            "uavs": [
                {
                    "uav_id": "A01",
                    "group_id": "GroupA",
                    "role": "Recon",
                    "capabilities": ["MOVE_TO"],
                },
                {
                    "uav_id": "A01",
                    "group_id": "GroupA",
                    "role": "Recon",
                    "capabilities": ["HOVER"],
                },
            ]
        },
    )
    with pytest.raises(ConfigurationLoadError, match="duplicate uav_id"):
        load_json_model(duplicate_uavs, UavCatalog)

    duplicate_plans = write_json(
        tmp_path / "duplicate-plans.json",
        {
            "plans": [
                {
                    "plan_id": "p1",
                    "group_id": "GroupA",
                    "robot_assignments": {
                        "A01": {"robot_id": "A01", "command_type": "MOVE_TO"}
                    },
                },
                {
                    "plan_id": "p1",
                    "group_id": "GroupA",
                    "robot_assignments": {
                        "A01": {"robot_id": "A01", "command_type": "HOVER"}
                    },
                },
            ]
        },
    )
    with pytest.raises(ConfigurationLoadError, match="duplicate plan_id"):
        load_json_model(duplicate_plans, PlanCatalog)


def test_loader_rejects_missing_state_and_cross_group_roster(tmp_path):
    robots_path, mission_path, plans_path = valid_files(tmp_path)
    mission = json.loads(mission_path.read_text(encoding="utf-8"))
    mission["initial_states"] = [
        {"uav_id": "A01", "online": True, "available": True}
    ]
    write_json(mission_path, mission)
    with pytest.raises(ConfigurationLoadError, match="one-to-one"):
        load_mission_context(robots_path, mission_path, plans_path)

    mission.pop("initial_states")
    mission["roster"]["active_ids"] = ["A01", "B01"]
    write_json(mission_path, mission)
    with pytest.raises(ConfigurationLoadError, match=r"unknown=\['B01'\]"):
        load_mission_context(robots_path, mission_path, plans_path)


def test_loader_rejects_plan_assignment_to_unknown_uav(tmp_path):
    robots_path, mission_path, plans_path = valid_files(tmp_path)
    plans = json.loads(plans_path.read_text(encoding="utf-8"))
    plans["plans"][0]["robot_assignments"] = {
        "A99": {"robot_id": "A99", "command_type": "MOVE_TO"}
    }
    write_json(plans_path, plans)

    with pytest.raises(ConfigurationLoadError, match="unknown or cross-Group"):
        load_mission_context(robots_path, mission_path, plans_path)


def test_loader_rejects_plan_assignment_lacking_capability(tmp_path):
    robots_path, mission_path, plans_path = valid_files(tmp_path)
    plans = json.loads(plans_path.read_text(encoding="utf-8"))
    plans["plans"][0]["robot_assignments"]["A01"]["command_type"] = "ATTACK"
    write_json(plans_path, plans)
    with pytest.raises(ConfigurationLoadError, match="lacks capability 'ATTACK'"):
        load_mission_context(robots_path, mission_path, plans_path)
