from pathlib import Path

import pytest

from uav_bt_runtime.bt import NodeStatus
from uav_bt_runtime.package import load_mission_package
from uav_bt_runtime.spatial import FieldConfig
from uav_bt_runtime.visual_simulation import (
    JointVisualSimulation,
    TkMissionViewer,
    _rectangles_overlap,
)


EXAMPLE_ROOT = Path(__file__).resolve().parents[1] / "examples" / "joint_mission"


def _simulation(scenario: str = "normal") -> JointVisualSimulation:
    return JointVisualSimulation(
        load_mission_package(EXAMPLE_ROOT / "group_a", require_reviewed=False),
        load_mission_package(EXAMPLE_ROOT / "group_b", require_reviewed=False),
        scenario=scenario,
        field=FieldConfig(width_m=100.0, height_m=150.0),
        uav_speed_mps=30.0,
        max_ticks=5000,
    )


def test_large_viewer_layout_keeps_map_and_status_panel_separate() -> None:
    layout = TkMissionViewer.layout_for_size(1900, 1010)
    plot_right = layout.plot_left + layout.plot_width
    plot_bottom = layout.plot_top + layout.plot_height
    assert plot_right + 20 < layout.panel_left
    assert layout.panel_left + layout.panel_width < layout.canvas_width
    assert plot_bottom + 20 < layout.canvas_height
    assert layout.panel_bottom < layout.canvas_height
    assert layout.panel_width >= 1000
    assert layout.plot_width / layout.plot_height == pytest.approx(
        100.0 / 150.0
    )


def test_viewer_layout_expands_when_user_resizes_the_window() -> None:
    normal = TkMissionViewer.layout_for_size(1600, 900)
    large = TkMissionViewer.layout_for_size(2200, 1250)
    assert large.canvas_width > normal.canvas_width
    assert large.canvas_height > normal.canvas_height
    assert large.plot_width > normal.plot_width
    assert large.plot_height > normal.plot_height
    assert large.panel_width > normal.panel_width
    assert large.plot_width / large.plot_height == pytest.approx(100.0 / 150.0)


def test_label_overlap_helper_respects_padding() -> None:
    assert not _rectangles_overlap((0, 0, 10, 10), (12, 0, 22, 10))
    assert _rectangles_overlap((0, 0, 10, 10), (12, 0, 22, 10), padding=3)


def test_scene_coordinates_show_all_31_nodes_in_agreed_start_areas() -> None:
    simulation = _simulation()
    try:
        assert simulation.mapper.scale == pytest.approx(1.0)
        assert simulation.mapper.offset_x == pytest.approx(0.0)
        assert simulation.mapper.offset_y == pytest.approx(0.0)
        group_a = list(simulation.transports["GroupA"].uavs.values())
        group_b = list(simulation.transports["GroupB"].uavs.values())
        assert len(group_a) == 15 and len(group_b) == 16
        assert all(5.0 <= uav.pose.x <= 35.0 for uav in group_a)
        assert all(5.0 <= uav.pose.y <= 15.0 for uav in group_a)
        assert all(80.0 <= uav.pose.x <= 95.0 for uav in group_b)
        assert all(5.0 <= uav.pose.y <= 20.0 for uav in group_b)
        assert all(simulation.all_uavs[f"B{i:02d}"].pose.z == 0.0 for i in range(7, 17))
    finally:
        simulation.close()


def test_normal_visual_simulation_follows_new_joint_flow() -> None:
    simulation = _simulation()
    try:
        result = simulation.run_to_completion()
        assert result.success
        assert result.group_a_status == NodeStatus.SUCCESS
        assert result.group_b_status == NodeStatus.SUCCESS
        assert result.attacks_completed == ("B01", "B02")

        context_a = simulation.packages["GroupA"].context
        assert context_a.roster.failed_ids == [
            "A07", "A08", "A09", "A10", "A11", "A12"
        ]
        assert {"A13", "A14", "A15"}.issubset(context_a.roster.active_ids)
        assert all(
            simulation.all_uavs[robot_id].pose.z == pytest.approx(8.0)
            for robot_id in context_a.roster.failed_ids
        )
        assert all(
            130.0 <= simulation.all_uavs[robot_id].pose.y <= 145.0
            for robot_id in context_a.roster.active_ids
        )
    finally:
        simulation.close()


def test_strike_failure_stops_both_groups() -> None:
    simulation = _simulation("strike-failure")
    try:
        result = simulation.run_to_completion()
        assert not result.success
        assert result.attacks_completed == ()
        strike_commands = [
            command
            for command in simulation.transports["GroupB"].history
            if command.command_type == "ATTACK"
        ]
        assert [command.robot_id for command in strike_commands] == ["B01", "B02"]
    finally:
        simulation.close()


def test_partial_damage_failure_preserves_completed_fault_exits() -> None:
    simulation = _simulation("partial-damage-failure")
    try:
        result = simulation.run_to_completion()
        assert not result.success
        context_a = simulation.packages["GroupA"].context
        assert context_a.roster.failed_ids == ["A07", "A08", "A09", "A10", "A11"]
        assert simulation.all_uavs["A07"].pose.z == pytest.approx(8.0)
        assert simulation.all_uavs["A12"].motion_state == "SAFE_HOLD"
    finally:
        simulation.close()
