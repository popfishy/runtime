"""Tests for the traditional runtime/main.py entry point."""

import importlib.util
from pathlib import Path


MAIN_PATH = Path(__file__).resolve().parents[1] / "main.py"
SPEC = importlib.util.spec_from_file_location("uav_runtime_outer_main", MAIN_PATH)
assert SPEC is not None and SPEC.loader is not None
OUTER_MAIN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(OUTER_MAIN)


def test_outer_main_validates_default_joint_packages(capsys):
    assert OUTER_MAIN.main(["--mode", "validate", "--allow-unreviewed"]) == 0
    assert '"status": "VALID"' in capsys.readouterr().out


def test_outer_main_runs_default_group_a_with_auto_peer_events(capsys):
    assert OUTER_MAIN.main([
        "--mode", "single", "--auto-peer-events", "--allow-unreviewed"
    ]) == 0
    output = capsys.readouterr().out
    assert '"group_id": "GroupA"' in output
    assert '"status": "SUCCESS"' in output


def test_outer_main_runs_default_joint_scenario(capsys):
    assert OUTER_MAIN.main([
        "--mode", "joint", "--scenario", "normal", "--allow-unreviewed"
    ]) == 0
    assert '"success": true' in capsys.readouterr().out


def test_outer_main_runs_headless_visual_scenario(capsys):
    assert OUTER_MAIN.main([
        "--mode", "visual", "--headless", "--allow-unreviewed"
    ]) == 0
    output = capsys.readouterr().out
    assert '"success": true' in output
    assert '"attacks_completed": ["B01", "B02"]' in output
