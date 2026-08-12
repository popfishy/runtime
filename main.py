"""Traditional script entry point for the UAV behavior-tree runtime."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List, Optional


RUNTIME_ROOT = Path(__file__).resolve().parent
SOURCE_ROOT = RUNTIME_ROOT / "src"
EXAMPLE_ROOT = RUNTIME_ROOT / "examples" / "joint_mission"

# Allow ``python main.py`` without installing the package or setting PYTHONPATH.
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from uav_bt_runtime.cli import main as runtime_cli_main  # noqa: E402
from uav_bt_runtime.package import load_mission_package  # noqa: E402
from uav_bt_runtime.spatial import FieldConfig  # noqa: E402
from uav_bt_runtime.visual_simulation import (  # noqa: E402
    JointVisualSimulation,
    TkMissionViewer,
)


SCENARIOS = [
    "normal",
    "strike-failure",
    "recovery-failure",
    "partial-damage-failure",
    "coordination-loss",
    "return-failure",
]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="四旋翼联合任务行为树传统启动程序"
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=["validate", "single", "joint", "visual", "tcp"],
        help=(
            "validate=检查任务包，single=运行单组，joint=运行A/B联合任务，"
            "visual=打开二维场景动画，tcp=连接本组TCP地面站"
        ),
    )
    parser.add_argument(
        "--package",
        type=Path,
        help="single模式的任务包；tcp模式必填；single默认使用示例Group A",
    )
    parser.add_argument(
        "--group-a",
        type=Path,
        default=EXAMPLE_ROOT / "group_a",
        help="Group A任务包",
    )
    parser.add_argument(
        "--group-b",
        type=Path,
        default=EXAMPLE_ROOT / "group_b",
        help="Group B任务包",
    )
    parser.add_argument(
        "--scenario",
        choices=SCENARIOS,
        default="normal",
        help="joint或visual模式使用的模拟场景",
    )
    parser.add_argument(
        "--auto-peer-events",
        action="store_true",
        help="single模式自动补齐对端阶段事件，使单组任务可以完整成功",
    )
    parser.add_argument(
        "--peer-event",
        action="append",
        default=[],
        help="single模式手动注入一个对端事件，可以重复使用",
    )
    parser.add_argument("--max-ticks", type=int, default=2500)
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument("--accept-timeout", type=float, default=5.0)
    parser.add_argument("--tcp-host", help="tcp模式使用的本组地面站IP或主机名")
    parser.add_argument("--tcp-port", type=int, default=39001)
    parser.add_argument("--tick-hz", type=float, default=10.0)
    parser.add_argument(
        "--log",
        type=Path,
        help="single模式为JSONL文件，joint/visual模式为日志目录",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="visual模式不打开窗口，只快速运行运动仿真并输出结果",
    )
    parser.add_argument(
        "--allow-unreviewed",
        action="store_true",
        help="仅用于候选任务包验证/仿真；实物运行前必须完成审核",
    )
    parser.add_argument("--field-width", type=float, default=100.0)
    parser.add_argument("--field-height", type=float, default=150.0)
    parser.add_argument("--tick-seconds", type=float, default=0.2)
    parser.add_argument("--uav-speed", type=float, default=12.0)
    parser.add_argument("--playback-speed", type=float, default=4.0)
    return parser


def _single_peer_events(
    package_path: Path, automatic: bool, allow_unreviewed: bool = False
) -> List[str]:
    if not automatic:
        return []
    package = load_mission_package(
        package_path, require_reviewed=not allow_unreviewed
    )
    if package.context.group_id == "GroupA":
        return ["GROUP_READY", "STRIKE_COMPLETE"]
    if package.context.group_id == "GroupB":
        return ["GROUP_READY", "RECON_COMPLETE"]
    raise ValueError(f"不支持的Group：{package.context.group_id}")


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.mode == "validate":
        if args.package is not None:
            command = ["validate", str(args.package)]
        else:
            command = ["validate", str(args.group_a), str(args.group_b)]
        if args.allow_unreviewed:
            command.append("--allow-unreviewed")
        return runtime_cli_main(command)

    if args.mode == "single":
        package_path = args.package or args.group_a
        peer_events = list(args.peer_event)
        peer_events.extend(
            event
            for event in _single_peer_events(
                package_path, args.auto_peer_events, args.allow_unreviewed
            )
            if event not in peer_events
        )
        command = [
            "run",
            "--package",
            str(package_path),
            "--max-ticks",
            str(args.max_ticks),
        ]
        for event in peer_events:
            command.extend(["--peer-event", event])
        if args.log is not None:
            command.extend(["--log", str(args.log)])
        if args.allow_unreviewed:
            command.append("--allow-unreviewed")
        return runtime_cli_main(command)

    if args.mode == "visual":
        return _run_visual(args)

    if args.mode == "tcp":
        if not args.tcp_host:
            raise ValueError("tcp模式必须提供--tcp-host")
        if args.package is None:
            raise ValueError("tcp模式必须显式提供--package")
        package_path = args.package
        peer_events = list(args.peer_event)
        peer_events.extend(
            event
            for event in _single_peer_events(
                package_path, args.auto_peer_events, args.allow_unreviewed
            )
            if event not in peer_events
        )
        command = [
            "tcp-run",
            "--package",
            str(package_path),
            "--tcp-host",
            args.tcp_host,
            "--tcp-port",
            str(args.tcp_port),
            "--max-ticks",
            str(args.max_ticks),
            "--connect-timeout",
            str(args.connect_timeout),
            "--accept-timeout",
            str(args.accept_timeout),
            "--tick-hz",
            str(args.tick_hz),
        ]
        for event in peer_events:
            command.extend(["--peer-event", event])
        if args.log is not None:
            command.extend(["--log", str(args.log)])
        if args.allow_unreviewed:
            command.append("--allow-unreviewed")
        return runtime_cli_main(command)

    command = [
        "simulate-joint",
        "--group-a",
        str(args.group_a),
        "--group-b",
        str(args.group_b),
        "--scenario",
        args.scenario,
        "--max-ticks",
        str(args.max_ticks),
    ]
    if args.log is not None:
        command.extend(["--log-dir", str(args.log)])
    if args.allow_unreviewed:
        command.append("--allow-unreviewed")
    return runtime_cli_main(command)


def _run_visual(args: argparse.Namespace) -> int:
    group_a = load_mission_package(
        args.group_a, require_reviewed=not args.allow_unreviewed
    )
    group_b = load_mission_package(
        args.group_b, require_reviewed=not args.allow_unreviewed
    )
    simulation = JointVisualSimulation(
        group_a=group_a,
        group_b=group_b,
        scenario=args.scenario,
        field=FieldConfig(args.field_width, args.field_height),
        tick_seconds=args.tick_seconds,
        max_ticks=args.max_ticks,
        uav_speed_mps=args.uav_speed,
        log_directory=args.log,
    )
    try:
        if args.headless:
            result = simulation.run_to_completion()
        else:
            try:
                result = TkMissionViewer(
                    simulation, playback_speed=args.playback_speed
                ).run()
            except Exception as exc:
                if type(exc).__name__ != "TclError":
                    raise
                print(
                    json.dumps(
                        {
                            "status": "ERROR",
                            "error": (
                                "无法打开图形窗口，请在桌面终端运行，或增加 "
                                "--headless 只验证运动仿真"
                            ),
                        },
                        ensure_ascii=False,
                    ),
                    file=sys.stderr,
                )
                return 2
    finally:
        simulation.close()
    print(
        json.dumps(
            {
                "scenario": args.scenario,
                "success": result.success,
                "group_a_status": result.group_a_status.value,
                "group_b_status": result.group_b_status.value,
                "ticks": result.ticks,
                "simulated_seconds": round(result.simulated_seconds, 1),
                "group_a_commands": result.group_a_commands,
                "group_b_commands": result.group_b_commands,
                "attacks_completed": list(result.attacks_completed),
            },
            ensure_ascii=False,
        )
    )
    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
