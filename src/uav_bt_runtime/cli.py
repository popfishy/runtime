"""Command-line entry points for validation and in-memory execution."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import List, Optional, Sequence

from .bt import NodeStatus
from .clock import ManualClock, SystemClock
from .executor import MissionExecutor, RuntimeServices
from .mission_log import make_logger
from .models import CoordinationEvent
from .package import (
    MissionPackageError,
    load_mission_package,
    validate_joint_packages,
)
from .simulation import run_joint_simulation
from .transport import (
    InMemoryCommandTransport,
    InMemoryCoordinationBus,
    InMemoryCoordinationTransport,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="uav-bt-runtime",
        description="Validate and simulate reviewed UAV behavior-tree mission packages.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate = subparsers.add_parser("validate", help="validate one or two mission packages")
    validate.add_argument("packages", nargs="+", type=Path)
    validate.add_argument("--allow-unreviewed", action="store_true")

    simulate = subparsers.add_parser(
        "simulate-joint", help="run both reviewed packages with in-memory transports"
    )
    simulate.add_argument("--group-a", type=Path, required=True)
    simulate.add_argument("--group-b", type=Path, required=True)
    simulate.add_argument(
        "--scenario",
        default="normal",
        choices=[
            "normal",
            "strike-failure",
            "recovery-failure",
            "partial-damage-failure",
            "coordination-loss",
            "return-failure",
        ],
    )
    simulate.add_argument("--max-ticks", type=int, default=1000)
    simulate.add_argument("--log-dir", type=Path)
    simulate.add_argument("--allow-unreviewed", action="store_true")

    run = subparsers.add_parser(
        "run", help="run one package against the in-memory command backend"
    )
    run.add_argument("--package", type=Path, required=True)
    run.add_argument("--peer-event", action="append", default=[])
    run.add_argument("--max-ticks", type=int, default=1000)
    run.add_argument("--log", type=Path)
    run.add_argument("--allow-unreviewed", action="store_true")

    tcp_run = subparsers.add_parser(
        "tcp-run", help="run one Group package against its TCP ground station"
    )
    tcp_run.add_argument("--package", type=Path, required=True)
    tcp_run.add_argument("--tcp-host", required=True)
    tcp_run.add_argument("--tcp-port", type=int, default=39001)
    tcp_run.add_argument("--peer-event", action="append", default=[])
    tcp_run.add_argument("--connect-timeout", type=float, default=10.0)
    tcp_run.add_argument("--accept-timeout", type=float, default=5.0)
    tcp_run.add_argument("--tick-hz", type=float, default=10.0)
    tcp_run.add_argument("--max-ticks", type=int, default=5000)
    tcp_run.add_argument("--log", type=Path)
    tcp_run.add_argument("--allow-unreviewed", action="store_true")
    return parser


def _validate_command(packages: Sequence[Path], *, allow_unreviewed: bool = False) -> int:
    if len(packages) not in {1, 2}:
        raise ValueError("validate accepts exactly one or two package directories")
    loaded = [
        load_mission_package(path, require_reviewed=not allow_unreviewed)
        for path in packages
    ]
    if len(loaded) == 2:
        validate_joint_packages(loaded[0], loaded[1])
    print(
        json.dumps(
            {
                "status": "VALID",
                "packages": [
                    {
                        "mission_id": package.context.mission_id,
                        "group_id": package.context.group_id,
                        "uav_count": len(package.context.uav_specs),
                        "tree_id": package.parsed_tree.tree.tree_id,
                    }
                    for package in loaded
                ],
            },
            ensure_ascii=False,
        )
    )
    return 0


def _simulate_command(args: argparse.Namespace) -> int:
    group_a = load_mission_package(
        args.group_a, require_reviewed=not args.allow_unreviewed
    )
    group_b = load_mission_package(
        args.group_b, require_reviewed=not args.allow_unreviewed
    )
    result = run_joint_simulation(
        group_a,
        group_b,
        scenario=args.scenario,
        max_ticks=args.max_ticks,
        log_directory=args.log_dir,
    )
    print(
        json.dumps(
            {
                "scenario": args.scenario,
                "success": result.success,
                "group_a_status": result.group_a_status.value,
                "group_b_status": result.group_b_status.value,
                "ticks": result.ticks,
                "group_a_commands": result.group_a_commands,
                "group_b_commands": result.group_b_commands,
            },
            ensure_ascii=False,
        )
    )
    return 0 if result.success else 1


def _run_command(args: argparse.Namespace) -> int:
    package = load_mission_package(
        args.package, require_reviewed=not args.allow_unreviewed
    )
    clock = ManualClock()
    bus = InMemoryCoordinationBus()
    group_id = package.context.group_id
    peer_group = "GroupB" if group_id == "GroupA" else "GroupA"
    local_coordination = InMemoryCoordinationTransport(group_id, bus)
    peer_coordination = InMemoryCoordinationTransport(peer_group, bus)
    for index, event_type in enumerate(args.peer_event):
        peer_coordination.publish(
            CoordinationEvent(
                mission_id=package.context.mission_id,
                event_id=f"cli-event-{index}",
                source_group=peer_group,
                event_type=event_type,
                sequence_number=index,
                timestamp=clock.time(),
            )
        )

    command_transport = InMemoryCommandTransport(group_id, clock)
    services = RuntimeServices(
        mission=package.context,
        command_transport=command_transport,
        coordination_transport=local_coordination,
        clock=clock,
        logger=make_logger(args.log, clock),
    )
    executor = MissionExecutor(package.parsed_tree.tree, services)
    terminal = {NodeStatus.SUCCESS, NodeStatus.FAILURE, NodeStatus.CANCELLED}
    ticks = 0
    interrupted = False
    try:
        while executor.status not in terminal and ticks < args.max_ticks:
            executor.tick()
            ticks += 1
            clock.advance(0.1)
        if executor.status not in terminal:
            executor.abort("MAX_TICKS_EXCEEDED")
    except KeyboardInterrupt:
        interrupted = True
        executor.cancel("OPERATOR_INTERRUPT")
    finally:
        status = executor.status
        executor.close()
        peer_coordination.close()
    print(
        json.dumps(
            {
                "mission_id": package.context.mission_id,
                "group_id": group_id,
                "status": status.value,
                "ticks": ticks,
                "commands": len(command_transport.history),
            },
            ensure_ascii=False,
        )
    )
    if interrupted:
        return 130
    return 0 if status == NodeStatus.SUCCESS else 1


def _tcp_run_command(args: argparse.Namespace) -> int:
    """Run one Group tree against one persistent newline-delimited TCP link."""

    if args.tick_hz <= 0:
        raise ValueError("tick_hz must be positive")
    if args.max_ticks <= 0:
        raise ValueError("max_ticks must be positive")

    from .tcp_transport import TcpCommandTransport

    package = load_mission_package(
        args.package, require_reviewed=not args.allow_unreviewed
    )
    clock = SystemClock()
    bus = InMemoryCoordinationBus()
    group_id = package.context.group_id
    peer_group = "GroupB" if group_id == "GroupA" else "GroupA"
    local_coordination = InMemoryCoordinationTransport(group_id, bus)
    peer_coordination = InMemoryCoordinationTransport(peer_group, bus)
    for index, event_type in enumerate(args.peer_event):
        peer_coordination.publish(
            CoordinationEvent(
                mission_id=package.context.mission_id,
                event_id=f"tcp-{group_id.lower()}-cli-event-{index}",
                source_group=peer_group,
                event_type=event_type,
                sequence_number=index,
                timestamp=clock.time(),
            )
        )

    command_transport = TcpCommandTransport(
        package=package,
        clock=clock,
        host=args.tcp_host,
        port=args.tcp_port,
        connect_timeout_s=args.connect_timeout,
        accept_timeout_s=args.accept_timeout,
    )
    command_transport.wait_until_ready()
    services = RuntimeServices(
        mission=package.context,
        command_transport=command_transport,
        coordination_transport=local_coordination,
        clock=clock,
        logger=make_logger(args.log, clock),
    )
    executor = MissionExecutor(package.parsed_tree.tree, services)
    terminal = {NodeStatus.SUCCESS, NodeStatus.FAILURE, NodeStatus.CANCELLED}
    ticks = 0
    interrupted = False
    try:
        period = 1.0 / args.tick_hz
        while executor.status not in terminal and ticks < args.max_ticks:
            executor.tick()
            ticks += 1
            if executor.status not in terminal:
                time.sleep(period)
        if executor.status not in terminal:
            executor.abort("MAX_TICKS_EXCEEDED")
    except KeyboardInterrupt:
        interrupted = True
        executor.cancel("OPERATOR_INTERRUPT")
    finally:
        status = executor.status
        executor.close()
        peer_coordination.close()

    print(
        json.dumps(
            {
                "mission_id": package.context.mission_id,
                "group_id": package.context.group_id,
                "tcp_ground_station": f"{args.tcp_host}:{args.tcp_port}",
                "status": status.value,
                "ticks": ticks,
                "commands": len(command_transport.history),
            },
            ensure_ascii=False,
        )
    )
    if interrupted:
        return 130
    return 0 if status == NodeStatus.SUCCESS else 1


def main(argv: Optional[List[str]] = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "validate":
            return _validate_command(
                args.packages, allow_unreviewed=args.allow_unreviewed
            )
        if args.command == "simulate-joint":
            return _simulate_command(args)
        if args.command == "run":
            return _run_command(args)
        if args.command == "tcp-run":
            return _tcp_run_command(args)
        raise ValueError(f"unknown command {args.command!r}")
    except (MissionPackageError, RuntimeError, ValueError) as exc:
        print(json.dumps({"status": "ERROR", "error": str(exc)}), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print(json.dumps({"status": "CANCELLED"}), file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
