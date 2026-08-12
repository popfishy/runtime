"""Prove that checked-in Leader routes contain sparse Fields2Cover swaths."""

import json
import math
import subprocess
from pathlib import Path

from runtime.tools.generate_example_mission import (
    formation_aware_swath_spacing,
    formation_scan_width,
)


RUNTIME_ROOT = Path(__file__).resolve().parents[1]
WORKER = RUNTIME_ROOT / "tools" / "fields2cover_worker.py"
PLANS = RUNTIME_ROOT / "examples" / "joint_mission" / "group_a" / "plans" / "plans.json"
FORMATION_SCAN_WIDTH_M = 15.0
SWATH_SPACING_M = 14.0


def test_coverage_width_uses_the_whole_cross_track_formation():
    initial_width = formation_scan_width(robot_count=12, columns=4)
    recovered_width = formation_scan_width(robot_count=9, columns=3)
    assert initial_width == FORMATION_SCAN_WIDTH_M
    assert recovered_width == FORMATION_SCAN_WIDTH_M
    assert formation_aware_swath_spacing(140.0, initial_width) == (
        10,
        SWATH_SPACING_M,
    )


def run_worker():
    completed = subprocess.run(
        [
            "/usr/bin/python3",
            str(WORKER),
            "--min-x", "15", "--max-x", "80",
            "--min-y", "5", "--max-y", "145",
            "--coverage-width", str(SWATH_SPACING_M),
            "--start-variant", "1",
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    result = json.loads(completed.stdout.strip().splitlines()[-1])
    assert result["ok"] is True
    assert result["generator"] == "fields2cover"
    assert result["ordering"] == "boustrophedon"
    return result


def test_fields2cover_worker_is_deterministic_sparse_and_written_to_plan():
    first_result = run_worker()
    second_result = run_worker()
    assert first_result == second_result
    first = first_result["points"]
    assert first_result["swath_count"] == 10
    assert len(first) == first_result["swath_count"] * 2
    assert len(first) == 20
    assert all(
        math.isfinite(float(point[key]))
        for point in first
        for key in ("x", "y", "yaw")
    )
    assert all(5.0 <= point["x"] <= 95.0 and 5.0 <= point["y"] <= 145.0 for point in first)

    # Every two points are one scan swath.  Boustrophedon order progresses
    # upward and alternates the horizontal flight direction.
    swath_y = []
    directions = []
    for index in range(0, len(first), 2):
        start, end = first[index:index + 2]
        assert start["y"] == end["y"]
        swath_y.append(start["y"])
        directions.append(math.copysign(1.0, end["x"] - start["x"]))
    assert swath_y == sorted(swath_y)
    assert swath_y == [
        12.0,
        26.0,
        40.0,
        54.0,
        68.0,
        82.0,
        96.0,
        110.0,
        124.0,
        138.0,
    ]
    assert all(
        math.isclose(right - left, SWATH_SPACING_M)
        for left, right in zip(swath_y, swath_y[1:])
    )
    assert FORMATION_SCAN_WIDTH_M - SWATH_SPACING_M == 1.0
    assert all(left != right for left, right in zip(directions, directions[1:]))

    catalog = json.loads(PLANS.read_text(encoding="utf-8"))
    segment1 = next(
        plan for plan in catalog["plans"] if plan["plan_id"] == "coverage-segment-1"
    )
    segment2 = next(
        plan for plan in catalog["plans"] if plan["plan_id"] == "coverage-segment-2"
    )
    stored1 = segment1["robot_assignments"]["A01"]["payload"]["waypoints"]
    stored2 = segment2["robot_assignments"]["A01"]["payload"]["waypoints"]
    assert len(stored1) == 11
    assert len(stored2) == 11
    assert stored1[-1]["y"] == 75.0
    assert stored2[0] == stored1[-1]
    assert [
        {"x": item["x"], "y": item["y"], "yaw": item["yaw"]}
        for item in stored1[:-1]
    ] == first[:10]
    assert [
        {"x": item["x"], "y": item["y"], "yaw": item["yaw"]}
        for item in stored2[1:]
    ] == first[10:]
