"""Generate the 31-UAV outdoor demonstration mission packages.

Coverage swaths are generated offline with the installed ``fields2cover``
Python package and stored as sparse endpoints in JSON.  The behavior-tree
runtime never imports the route generator.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple


RUNTIME_ROOT = Path(__file__).resolve().parents[1]
ROOT = RUNTIME_ROOT / "examples" / "joint_mission"
WORKER = Path(__file__).with_name("fields2cover_worker.py")
MISSION_ID = "joint-mission-002"
TARGET_VERSION = "target-v3"
FRAME_ID = "mission_enu"
ALTITUDE_M = 12.0
FAULT_ALTITUDE_M = 8.0
FORMATION_SPACING_M = 5.0
UAV_SCAN_WIDTH_M = 5.0
COVERAGE_SPLIT_Y_M = 75.0
FLIGHT_AREA = {
    "min_x": 0.0,
    "max_x": 100.0,
    "min_y": 0.0,
    "max_y": 150.0,
    "safety_margin_m": 5.0,
}


def write_json(path: Path, data: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.strip() + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pose(x: float, y: float, z: float = ALTITUDE_M, yaw: float = 0.0) -> Dict[str, float]:
    return {
        "x": round(float(x), 4),
        "y": round(float(y), 4),
        "z": round(float(z), 3),
        "yaw": round(float(yaw), 6),
    }


def uav(
    uav_id: str,
    group_id: str,
    role: str,
    capabilities: Iterable[str],
    *,
    leader: bool = False,
    reserve: bool = False,
) -> Dict[str, object]:
    return {
        "uav_id": uav_id,
        "group_id": group_id,
        "role": role,
        "is_leader": leader,
        "is_reserve": reserve,
        "capabilities": sorted(set(capabilities)),
    }


def assignment(
    robot_id: str,
    command_type: str,
    payload: Mapping[str, object],
    terminal_action: str = "",
) -> Dict[str, object]:
    result: Dict[str, object] = {
        "robot_id": robot_id,
        "command_type": command_type,
        "payload": dict(payload),
    }
    if terminal_action:
        result["terminal_action"] = terminal_action
    return result


def plan(
    plan_id: str,
    group_id: str,
    assignments: Mapping[str, Mapping[str, object]],
) -> Dict[str, object]:
    return {
        "plan_id": plan_id,
        "group_id": group_id,
        "robot_assignments": dict(assignments),
        "completion_policy": "ALL",
    }


def grid_positions(
    ids: Sequence[str],
    *,
    center: Tuple[float, float],
    columns: int,
    spacing: float = FORMATION_SPACING_M,
    leader_id: str = "A01",
) -> Dict[str, Tuple[float, float]]:
    rows = int(math.ceil(len(ids) / float(columns)))
    min_x = center[0] - (columns - 1) * spacing / 2.0
    min_y = center[1] - (rows - 1) * spacing / 2.0
    ordered_ids = list(ids)
    if leader_id in ordered_ids:
        ordered_ids.remove(leader_id)
        center_index = (rows // 2) * columns + max(0, (columns - 1) // 2)
        center_index = min(center_index, len(ids) - 1)
        ordered_ids.insert(center_index, leader_id)
    return {
        robot_id: (
            min_x + (index % columns) * spacing,
            min_y + (index // columns) * spacing,
        )
        for index, robot_id in enumerate(ordered_ids)
    }


def formation_scan_width(robot_count: int, columns: int) -> float:
    """Return the cross-track footprint of a rectangular UAV formation."""

    if robot_count <= 0 or columns <= 0:
        raise ValueError("robot_count and columns must be positive")
    rows = int(math.ceil(robot_count / float(columns)))
    return (rows - 1) * FORMATION_SPACING_M + UAV_SCAN_WIDTH_M


def formation_aware_swath_spacing(
    scan_span_m: float, formation_width_m: float
) -> Tuple[int, float]:
    """Tile a scan span without gaps, allowing a small formation overlap."""

    if scan_span_m <= 0.0 or formation_width_m <= 0.0:
        raise ValueError("scan span and formation width must be positive")
    swath_count = int(math.ceil(scan_span_m / formation_width_m))
    return swath_count, scan_span_m / swath_count


def _generate_leader_route(
    *,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    coverage_width: float,
) -> List[Dict[str, float]]:
    python = os.environ.get("FIELDS2COVER_PYTHON", "/usr/bin/python3")
    command = [
        python,
        str(WORKER),
        "--min-x",
        str(min_x),
        "--max-x",
        str(max_x),
        "--min-y",
        str(min_y),
        "--max-y",
        str(max_y),
        "--coverage-width",
        str(coverage_width),
        "--start-variant",
        "1",
    ]
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    if completed.returncode != 0:
        raise RuntimeError(
            "Fields2Cover worker failed: "
            f"exit={completed.returncode}, stdout={completed.stdout!r}, "
            f"stderr={completed.stderr!r}"
        )
    try:
        result = json.loads(completed.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"invalid Fields2Cover worker output: {completed.stdout!r}") from exc
    if not result.get("ok"):
        raise RuntimeError(f"Fields2Cover worker error: {result.get('error', 'unknown error')}")

    safe_min_x = FLIGHT_AREA["min_x"] + FLIGHT_AREA["safety_margin_m"]
    safe_max_x = FLIGHT_AREA["max_x"] - FLIGHT_AREA["safety_margin_m"]
    safe_min_y = FLIGHT_AREA["min_y"] + FLIGHT_AREA["safety_margin_m"]
    safe_max_y = FLIGHT_AREA["max_y"] - FLIGHT_AREA["safety_margin_m"]
    route = []
    for index, item in enumerate(result["points"]):
        x, y, yaw = float(item["x"]), float(item["y"]), float(item["yaw"])
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise RuntimeError(f"leader route point {index} is not finite")
        if not safe_min_x <= x <= safe_max_x or not safe_min_y <= y <= safe_max_y:
            raise RuntimeError(f"leader route point {index} is outside the safe flight area")
        route.append(pose(x, y, ALTITUDE_M, yaw))
    if len(route) < 2:
        raise RuntimeError("leader coverage route needs at least two points")
    return route


def _validate_formation_envelope(
    route: Sequence[Mapping[str, float]],
    robot_ids: Sequence[str],
    *,
    columns: int,
    plan_id: str,
) -> None:
    """Reject an offline route if any formation member leaves the safe area."""

    slots = grid_positions(robot_ids, center=(0.0, 0.0), columns=columns)
    leader_x, leader_y = slots["A01"]
    offsets = {
        robot_id: (x - leader_x, y - leader_y)
        for robot_id, (x, y) in slots.items()
    }
    safe_min_x = FLIGHT_AREA["min_x"] + FLIGHT_AREA["safety_margin_m"]
    safe_max_x = FLIGHT_AREA["max_x"] - FLIGHT_AREA["safety_margin_m"]
    safe_min_y = FLIGHT_AREA["min_y"] + FLIGHT_AREA["safety_margin_m"]
    safe_max_y = FLIGHT_AREA["max_y"] - FLIGHT_AREA["safety_margin_m"]
    for point_index, waypoint in enumerate(route):
        leader_route_x = float(waypoint["x"])
        leader_route_y = float(waypoint["y"])
        for robot_id, (offset_x, offset_y) in offsets.items():
            member_x = leader_route_x + offset_x
            member_y = leader_route_y + offset_y
            if not (
                safe_min_x <= member_x <= safe_max_x
                and safe_min_y <= member_y <= safe_max_y
            ):
                raise RuntimeError(
                    f"{plan_id} point {point_index} places {robot_id} outside "
                    f"the safe flight area at ({member_x:.3f}, {member_y:.3f})"
                )


def _validate_progressive_coverage(
    segment1: Sequence[Mapping[str, float]],
    segment2: Sequence[Mapping[str, float]],
    *,
    split_y: float,
) -> None:
    """Ensure both segments form one open lower-to-upper coverage mission."""

    if len(segment1) < 3 or len(segment2) < 3:
        raise RuntimeError("progressive coverage segments are too short")
    first_end = segment1[-1]
    second_start = segment2[0]
    if any(
        not math.isclose(float(first_end[key]), float(second_start[key]), abs_tol=1e-6)
        for key in ("x", "y", "z")
    ):
        raise RuntimeError("coverage segment 2 must start at segment 1's endpoint")
    if not math.isclose(float(first_end["y"]), split_y, abs_tol=1e-6):
        raise RuntimeError("coverage segment 1 must end on the configured split line")

    first_scan = segment1[:-1]
    second_scan = segment2[1:]
    if any(float(point["y"]) > split_y + 1e-6 for point in first_scan):
        raise RuntimeError("coverage segment 1 enters the unscanned upper region")
    if any(float(point["y"]) < split_y - 1e-6 for point in second_scan):
        raise RuntimeError("coverage segment 2 returns to the scanned lower region")
    if float(first_scan[-1]["y"]) <= float(first_scan[0]["y"]):
        raise RuntimeError("coverage segment 1 does not progress toward the split line")
    if float(second_scan[-1]["y"]) <= float(second_scan[0]["y"]):
        raise RuntimeError("coverage segment 2 does not progress toward the upper boundary")


def formation_positions_at_leader(
    ids: Sequence[str],
    *,
    leader_position: Tuple[float, float],
    columns: int,
    leader_id: str = "A01",
) -> Dict[str, Tuple[float, float]]:
    """Resolve every member position when a route point represents the Leader."""

    slots = grid_positions(
        ids, center=(0.0, 0.0), columns=columns, leader_id=leader_id
    )
    leader_slot_x, leader_slot_y = slots[leader_id]
    return {
        robot_id: (
            leader_position[0] + slot_x - leader_slot_x,
            leader_position[1] + slot_y - leader_slot_y,
        )
        for robot_id, (slot_x, slot_y) in slots.items()
    }


def build_uavs() -> List[Dict[str, object]]:
    result: List[Dict[str, object]] = []
    for index in range(1, 16):
        robot_id = f"A{index:02d}"
        capabilities = {"MOVE_TO", "FOLLOW_ROUTE", "HOVER"}
        if 7 <= index <= 12:
            capabilities.add("FAULT_EXIT")
        result.append(
            uav(
                robot_id,
                "GroupA",
                "ReconReserve" if index >= 13 else "Recon",
                capabilities,
                leader=index == 1,
                reserve=index >= 13,
            )
        )
    for index in range(1, 17):
        robot_id = f"B{index:02d}"
        capabilities = {"HOVER"}
        if index <= 6:
            capabilities.update({"ATTACK", "RETURN"})
        result.append(
            uav(
                robot_id,
                "GroupB",
                "Strike" if index <= 6 else "GroundNode",
                capabilities,
                leader=index == 1,
            )
        )
    return result


def _leader_follow_assignments(
    plan_id: str,
    robot_ids: Sequence[str],
    leader_route: Sequence[Mapping[str, float]],
) -> Dict[str, Dict[str, object]]:
    del plan_id
    return {
        robot_id: assignment(
            robot_id,
            "FOLLOW_ROUTE",
            {"waypoints": list(leader_route)}
            if robot_id == "A01"
            else {"formation_follow": True},
            "HOVER",
        )
        for robot_id in robot_ids
    }


def group_a_plans(
    segment1: Sequence[Mapping[str, float]],
    segment2: Sequence[Mapping[str, float]],
) -> List[Dict[str, object]]:
    initial_ids = [f"A{index:02d}" for index in range(1, 13)]
    fault_ids = [f"A{index:02d}" for index in range(7, 13)]
    recovered_ids = [f"A{index:02d}" for index in range(1, 7)] + ["A13", "A14", "A15"]
    staging = grid_positions(initial_ids, center=(20.0, 50.0), columns=4)
    recovery_center = (float(segment2[0]["x"]), float(segment2[0]["y"]))
    recovery = grid_positions(recovered_ids, center=recovery_center, columns=3)
    hold_center = (float(segment2[-1]["x"]), float(segment2[-1]["y"]))
    hold = grid_positions(recovered_ids, center=hold_center, columns=3)

    segment1_end = (float(segment1[-1]["x"]), float(segment1[-1]["y"]))
    segment1_formation = formation_positions_at_leader(
        initial_ids, leader_position=segment1_end, columns=4
    )
    exit_points = {
        "A07": (5.0, 20.0),
        "A08": (10.0, 20.0),
        "A09": (15.0, 20.0),
        "A10": (5.0, 25.0),
        "A11": (10.0, 25.0),
        "A12": (15.0, 25.0),
    }
    fault_assignments = {}
    for robot_id in fault_ids:
        current_x, current_y = segment1_formation[robot_id]
        exit_x, exit_y = exit_points[robot_id]
        fault_assignments[robot_id] = assignment(
            robot_id,
            "FAULT_EXIT",
            {
                "waypoints": [
                    pose(current_x, current_y, FAULT_ALTITUDE_M),
                    pose(exit_x, exit_y, FAULT_ALTITUDE_M),
                ],
                "manual_landing_required": True,
            },
        )

    return [
        plan(
            "prepare-a",
            "GroupA",
            {
                robot_id: assignment(
                    robot_id,
                    "MOVE_TO",
                    {"target_pose": pose(*staging[robot_id])},
                    "HOVER",
                )
                for robot_id in initial_ids
            },
        ),
        plan(
            "coverage-segment-1",
            "GroupA",
            _leader_follow_assignments("coverage-segment-1", initial_ids, segment1),
        ),
        plan("fault-exit", "GroupA", fault_assignments),
        plan(
            "recovery-a",
            "GroupA",
            {
                robot_id: assignment(
                    robot_id,
                    "MOVE_TO",
                    {
                        "target_pose": pose(*recovery[robot_id]),
                        "manual_takeoff_required": robot_id in {"A13", "A14", "A15"},
                    },
                    "HOVER",
                )
                for robot_id in recovered_ids
            },
        ),
        plan(
            "coverage-segment-2",
            "GroupA",
            _leader_follow_assignments("coverage-segment-2", recovered_ids, segment2),
        ),
        plan(
            "hold-a-end",
            "GroupA",
            {
                robot_id: assignment(
                    robot_id,
                    "HOVER",
                    {"hold_pose": pose(*hold[robot_id])},
                )
                for robot_id in recovered_ids
            },
        ),
    ]


def group_b_plans() -> List[Dict[str, object]]:
    return_points = {"B01": (90.0, 10.0), "B02": (95.0, 10.0)}
    return [
        plan(
            "strike-targets",
            "GroupB",
            {
                "B01": assignment(
                    "B01",
                    "ATTACK",
                    {"target_id": "target-1", "manual_takeoff_required": True},
                ),
                "B02": assignment(
                    "B02",
                    "ATTACK",
                    {"target_id": "target-2", "manual_takeoff_required": True},
                ),
            },
        ),
        plan(
            "return-strike-uavs",
            "GroupB",
            {
                robot_id: assignment(
                    robot_id,
                    "RETURN",
                    {"waypoints": [pose(x, y)]},
                    "HOVER",
                )
                for robot_id, (x, y) in return_points.items()
            },
        ),
    ]


GROUP_A_TREE = """
<root main_tree_to_execute="GroupATree">
  <BehaviorTree ID="GroupATree">
    <Sequence>
      <Action ID="PrepareGroupA" plan_id="prepare-a" timeout_s="90"/>
      <Action ID="PublishGroupReady"/>
      <Action ID="WaitGroupBReady" timeout_s="120"/>
      <Action ID="CoverageSegment1" plan_id="coverage-segment-1" timeout_s="300"/>
      <Action ID="SimulateDamage" plan_id="fault-exit" timeout_s="180"/>
      <Action ID="RecoverReconGroup" plan_id="recovery-a" timeout_s="240"/>
      <Action ID="CoverageSegment2" plan_id="coverage-segment-2" timeout_s="300"/>
      <Action ID="PublishReconComplete"/>
      <Action ID="WaitStrikeComplete" timeout_s="300"/>
      <Action ID="HoldCoverageEnd" plan_id="hold-a-end" timeout_s="60"/>
    </Sequence>
  </BehaviorTree>
</root>
"""


GROUP_B_TREE = """
<root main_tree_to_execute="GroupBTree">
  <BehaviorTree ID="GroupBTree">
    <Sequence>
      <Action ID="PublishGroupReady"/>
      <Action ID="WaitGroupAReady" timeout_s="120"/>
      <Action ID="WaitReconComplete" timeout_s="600"/>
      <Action ID="StrikeTargets" plan_id="strike-targets" timeout_s="300"/>
      <Action ID="ReturnStrikeUavs" plan_id="return-strike-uavs" timeout_s="300"/>
      <Action ID="PublishStrikeComplete"/>
    </Sequence>
  </BehaviorTree>
</root>
"""


def build_package(
    group_id: str,
    package_version: str,
    peer_version: str,
    uavs: Sequence[Mapping[str, object]],
    plans: Sequence[Mapping[str, object]],
    *,
    reviewed: bool,
) -> None:
    directory = ROOT / ("group_a" if group_id == "GroupA" else "group_b")
    group_uavs = [item for item in uavs if item["group_id"] == group_id]
    if group_id == "GroupA":
        active_ids = [f"A{index:02d}" for index in range(1, 13)]
        reserve_ids = ["A13", "A14", "A15"]
        inactive_ids: List[str] = []
        planned_fault_count = 6
        planned_recovery_count = 3
    else:
        active_ids = [f"B{index:02d}" for index in range(1, 7)]
        reserve_ids = []
        inactive_ids = [f"B{index:02d}" for index in range(7, 17)]
        planned_fault_count = 0
        planned_recovery_count = 0

    world = {
        "schema_version": "1.0",
        "coordinate_frame": FRAME_ID,
        "target_config_version": TARGET_VERSION,
        "flight_area": FLIGHT_AREA,
        "targets": {
            "target-1": pose(65.0, 65.0),
            "target-2": pose(75.0, 105.0),
        },
    }
    initial_states = []
    for item in group_uavs:
        robot_id = str(item["uav_id"])
        inactive = robot_id in inactive_ids
        initial_states.append(
            {
                "uav_id": robot_id,
                "online": not inactive,
                "available": not inactive,
            }
        )

    write_json(directory / "robots.json", {"schema_version": "1.0", "uavs": list(uavs)})
    write_json(directory / "world.json", world)
    write_json(
        directory / "bootstrap.json",
        {
            "schema_version": "1.0",
            "mission_id": MISSION_ID,
            "group_id": group_id,
            "roster": {
                "group_id": group_id,
                "active_ids": active_ids,
                "reserve_ids": reserve_ids,
                "failed_ids": [],
                "inactive_ids": inactive_ids,
            },
            "initial_states": initial_states,
            "blackboard": {"mission_armed": True},
            "planned_fault_count": planned_fault_count,
            "planned_recovery_count": planned_recovery_count,
        },
    )
    write_json(directory / "plans" / "plans.json", {"schema_version": "1.0", "plans": list(plans)})
    write_text(directory / "tree.xml", GROUP_A_TREE if group_id == "GroupA" else GROUP_B_TREE)

    referenced = ["tree.xml", "robots.json", "world.json", "bootstrap.json", "plans/plans.json"]
    write_json(
        directory / "mission.json",
        {
            "schema_version": "1.0",
            "package_version": package_version,
            "peer_package_version": peer_version,
            "mission_id": MISSION_ID,
            "group_id": group_id,
            "reviewed": reviewed,
            "tree_file": "tree.xml",
            "robots_file": "robots.json",
            "world_file": "world.json",
            "bootstrap_file": "bootstrap.json",
            "plan_files": ["plans/plans.json"],
            "target_config_version": TARGET_VERSION,
            "file_hashes": {relative: sha256(directory / relative) for relative in referenced},
        },
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mark-reviewed",
        action="store_true",
        help="mark generated packages reviewed after a human has checked every route",
    )
    return parser


def main() -> None:
    args = _parser().parse_args()
    initial_ids = [f"A{index:02d}" for index in range(1, 13)]
    recovered_ids = [f"A{index:02d}" for index in range(1, 7)] + [
        "A13",
        "A14",
        "A15",
    ]
    initial_width = formation_scan_width(len(initial_ids), columns=4)
    recovered_width = formation_scan_width(len(recovered_ids), columns=3)
    if not math.isclose(initial_width, recovered_width, abs_tol=1e-6):
        raise RuntimeError(
            "coverage phases require the same cross-track formation width; "
            f"got {initial_width:.3f} m and {recovered_width:.3f} m"
        )

    safe_min_y = FLIGHT_AREA["min_y"] + FLIGHT_AREA["safety_margin_m"]
    safe_max_y = FLIGHT_AREA["max_y"] - FLIGHT_AREA["safety_margin_m"]
    expected_swaths, swath_spacing = formation_aware_swath_spacing(
        safe_max_y - safe_min_y, initial_width
    )
    if expected_swaths % 2:
        raise RuntimeError("coverage swaths must split evenly at the fault line")

    # Generate one continuous lower-to-upper swath sequence, then divide that
    # sequence at the mission midpoint.  The 3-row formation is 15 m wide
    # across-track; ten 14 m-spaced swaths cover the 140 m safe span with 1 m
    # overlap instead of treating every UAV as an independent 5 m swath.
    full_scan = _generate_leader_route(
        min_x=15.0,
        max_x=80.0,
        min_y=safe_min_y,
        max_y=safe_max_y,
        coverage_width=swath_spacing,
    )
    if len(full_scan) != expected_swaths * 2:
        raise RuntimeError(
            "Fields2Cover returned an unexpected swath count: "
            f"expected {expected_swaths}, got {len(full_scan) // 2}"
        )
    split_index = expected_swaths
    first_scan = full_scan[:split_index]
    second_scan = full_scan[split_index:]

    # Both phases meet at one explicit fault/recovery point.  Fields2Cover
    # supplies only ordered swath endpoints; the lower layer plans transitions
    # and turns between these sparse task points.
    split_point = pose(
        float(first_scan[-1]["x"]),
        COVERAGE_SPLIT_Y_M,
        yaw=math.pi / 2.0,
    )
    segment1 = list(first_scan) + [split_point]
    segment2 = [split_point] + list(second_scan)
    _validate_progressive_coverage(
        segment1, segment2, split_y=COVERAGE_SPLIT_Y_M
    )
    _validate_formation_envelope(
        segment1,
        initial_ids,
        columns=4,
        plan_id="coverage-segment-1",
    )
    _validate_formation_envelope(
        segment2,
        recovered_ids,
        columns=3,
        plan_id="coverage-segment-2",
    )
    uavs = build_uavs()
    build_package(
        "GroupA",
        "3.0-a",
        "3.0-b",
        uavs,
        group_a_plans(segment1, segment2),
        reviewed=args.mark_reviewed,
    )
    build_package(
        "GroupB",
        "3.0-b",
        "3.0-a",
        uavs,
        group_b_plans(),
        reviewed=args.mark_reviewed,
    )
    status = "reviewed" if args.mark_reviewed else "candidate (reviewed=false)"
    print(f"generated {status} mission packages under {ROOT}")


if __name__ == "__main__":
    main()
