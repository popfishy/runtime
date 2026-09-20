#!/usr/bin/env python3
"""Traditional entry point for executing one reviewed UAV task."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import List, Optional


RUNTIME_ROOT = Path(__file__).resolve().parent
SOURCE_ROOT = RUNTIME_ROOT / "src"
EXAMPLE_ROOT = RUNTIME_ROOT / "examples" / "joint_mission"

if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from uav_bt_runtime.bt import NodeStatus  # noqa: E402
from uav_bt_runtime.clock import ManualClock, SystemClock  # noqa: E402
from uav_bt_runtime.package import load_mission_package  # noqa: E402
from uav_bt_runtime.simulation import build_scripted_backend  # noqa: E402
from uav_bt_runtime.task_testing import (  # noqa: E402
    available_tasks,
    run_single_task,
    task_timeout_s,
)
from uav_bt_runtime.transport import (  # noqa: E402
    InMemoryCommandTransport,
    ScriptedBackend,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Group A/B 单任务测试程序")
    parser.add_argument("--group", required=True, choices=["A", "B", "a", "b"])
    parser.add_argument("--task", required=True, help="要单独执行的任务名称")
    parser.add_argument(
        "--backend",
        choices=["memory", "tcp"],
        default="memory",
        help="memory=快速单元测试，tcp=连接本组TCP地面站",
    )
    parser.add_argument("--package", type=Path, help="覆盖默认示例任务包路径")
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument("--accept-timeout", type=float, default=5.0)
    parser.add_argument("--tcp-host")
    parser.add_argument("--tcp-port", type=int, default=39001)
    parser.add_argument("--tick-hz", type=float, default=20.0)
    parser.add_argument(
        "--max-ticks",
        type=int,
        help="最大tick数；省略时按任务超时和tick频率自动计算",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        help="覆盖单任务命令及本地等待超时（秒）",
    )
    return parser


def _memory_backend(group_id: str, task: str) -> ScriptedBackend:
    del task
    return build_scripted_backend(group_id, "normal")


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    group_suffix = args.group.upper()
    group_id = f"Group{group_suffix}"
    if args.task not in available_tasks(group_id):
        print(
            json.dumps(
                {
                    "status": "ERROR",
                    "error": f"{group_id}不支持任务{args.task!r}",
                    "available_tasks": list(available_tasks(group_id)),
                },
                ensure_ascii=False,
            ),
            file=sys.stderr,
        )
        return 2
    package_path = args.package or EXAMPLE_ROOT / f"group_{group_suffix.lower()}"
    timeout_s = args.timeout_s
    if timeout_s is not None and timeout_s <= 0:
        raise ValueError("timeout-s must be positive")
    configured_timeout_s = timeout_s or task_timeout_s(group_id, args.task)
    max_ticks = args.max_ticks
    if max_ticks is None:
        max_ticks = math.ceil(configured_timeout_s * args.tick_hz) + 1
    if max_ticks <= 0:
        raise ValueError("max-ticks must be positive")

    transport = None
    try:
        # This is an explicit test utility: candidate packages may be exercised
        # before the human reviewer changes mission.json to reviewed=true.
        package = load_mission_package(package_path, require_reviewed=False)
        if package.context.group_id != group_id:
            raise ValueError(
                f"任务包属于{package.context.group_id}，但命令选择了{group_id}"
            )

        if args.backend == "memory":
            clock = ManualClock()
            transport = InMemoryCommandTransport(
                group_id, clock, _memory_backend(group_id, args.task)
            )
            result = run_single_task(
                package,
                args.task,
                transport,
                clock,
                max_ticks=max_ticks,
                timeout_s=timeout_s,
            )
            output = result.to_dict()
        else:
            if not args.tcp_host:
                raise ValueError("tcp backend必须提供--tcp-host")
            if args.tick_hz <= 0:
                raise ValueError("tick-hz must be positive")
            import time

            from uav_bt_runtime.tcp_transport import TcpCommandTransport

            clock = SystemClock()
            transport = TcpCommandTransport(
                package,
                clock,
                args.tcp_host,
                args.tcp_port,
                connect_timeout_s=args.connect_timeout,
                accept_timeout_s=args.accept_timeout,
            )
            transport.wait_until_ready()
            result = run_single_task(
                package,
                args.task,
                transport,
                clock,
                max_ticks=max_ticks,
                timeout_s=timeout_s,
                wait_for_next_tick=lambda: time.sleep(1.0 / args.tick_hz),
            )
            output = result.to_dict()
            output["tcp_commands"] = [item.to_dict() for item in transport.history]

        print(json.dumps(output, ensure_ascii=False, sort_keys=True))
        return 0 if result.status == NodeStatus.SUCCESS else 1
    except Exception as exc:
        print(
            json.dumps(
                {"group_id": group_id, "task": args.task, "status": "ERROR", "error": str(exc)},
                ensure_ascii=False,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2
    finally:
        if transport is not None:
            transport.close()


if __name__ == "__main__":
    raise SystemExit(main())
