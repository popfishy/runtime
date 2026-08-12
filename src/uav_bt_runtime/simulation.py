"""Deterministic two-Group execution using only in-memory transports."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Union

from .bt import NodeStatus
from .clock import ManualClock
from .executor import MissionExecutor, RuntimeServices
from .mission_log import MemoryMissionLogger, MissionLogger, make_logger
from .models import CommandStatus
from .package import MissionPackage, validate_joint_packages
from .transport import (
    InMemoryCommandTransport,
    InMemoryCoordinationBus,
    InMemoryCoordinationTransport,
    ScriptRule,
    ScriptedBackend,
    ScriptedOutcome,
)


@dataclass(frozen=True)
class JointSimulationResult:
    group_a_status: NodeStatus
    group_b_status: NodeStatus
    ticks: int
    group_a_commands: int
    group_b_commands: int

    @property
    def success(self) -> bool:
        return (
            self.group_a_status == NodeStatus.SUCCESS
            and self.group_b_status == NodeStatus.SUCCESS
        )


def build_scripted_backend(group_id: str, scenario: str) -> ScriptedBackend:
    """Build the deterministic failure rules shared by all simulators."""

    rules = []
    if scenario == "strike-failure" and group_id == "GroupB":
        rules.append(
            ScriptRule(
                plan_id="strike-targets",
                outcome=ScriptedOutcome(
                    status=CommandStatus.FAILURE,
                    error_code="SIMULATED_STRIKE_FAILURE",
                ),
            )
        )
    elif scenario == "recovery-failure" and group_id == "GroupA":
        rules.append(
            ScriptRule(
                plan_id="recovery-a",
                robot_id="A15",
                command_type="MOVE_TO",
                max_matches=1,
                outcome=ScriptedOutcome(
                    status=CommandStatus.FAILURE,
                    error_code="SIMULATED_RECOVERY_FAILURE",
                ),
            )
        )
    elif scenario == "partial-damage-failure" and group_id == "GroupA":
        rules.append(
            ScriptRule(
                plan_id="fault-exit",
                robot_id="A12",
                command_type="FAULT_EXIT",
                max_matches=1,
                outcome=ScriptedOutcome(
                    status=CommandStatus.FAILURE,
                    error_code="SIMULATED_PARTIAL_DAMAGE_FAILURE",
                ),
            )
        )
    elif scenario == "return-failure" and group_id == "GroupB":
        rules.append(
            ScriptRule(
                plan_id="return-strike-uavs",
                robot_id="B01",
                command_type="RETURN",
                max_matches=1,
                outcome=ScriptedOutcome(status=CommandStatus.FAILURE),
            )
        )
    return ScriptedBackend(rules=rules)


def _logger_for(
    group_id: str,
    clock: ManualClock,
    log_directory: Optional[Union[str, Path]],
) -> MissionLogger:
    if log_directory is None:
        return MemoryMissionLogger(clock)
    path = Path(log_directory) / f"{group_id}.jsonl"
    return make_logger(path, clock)


def run_joint_simulation(
    group_a: MissionPackage,
    group_b: MissionPackage,
    scenario: str = "normal",
    max_ticks: int = 1000,
    tick_seconds: float = 0.1,
    log_directory: Optional[Union[str, Path]] = None,
) -> JointSimulationResult:
    """Run both isolated executors against deterministic scripted feedback."""

    supported = {
        "normal",
        "strike-failure",
        "recovery-failure",
        "partial-damage-failure",
        "coordination-loss",
        "return-failure",
    }
    if scenario not in supported:
        raise ValueError(f"unsupported simulation scenario {scenario!r}")
    if max_ticks <= 0 or tick_seconds <= 0:
        raise ValueError("max_ticks and tick_seconds must be positive")
    validate_joint_packages(group_a, group_b)

    clock = ManualClock()
    dropped = ["RECON_COMPLETE"] if scenario == "coordination-loss" else []
    bus = InMemoryCoordinationBus(drop_event_types=dropped)

    packages: Dict[str, MissionPackage] = {
        group_a.context.group_id: group_a,
        group_b.context.group_id: group_b,
    }
    executors: Dict[str, MissionExecutor] = {}
    command_transports: Dict[str, InMemoryCommandTransport] = {}
    for group_id, package in packages.items():
        command_transport = InMemoryCommandTransport(
            group_id, clock, build_scripted_backend(group_id, scenario)
        )
        command_transports[group_id] = command_transport
        services = RuntimeServices(
            mission=package.context,
            command_transport=command_transport,
            coordination_transport=InMemoryCoordinationTransport(group_id, bus),
            clock=clock,
            logger=_logger_for(group_id, clock, log_directory),
        )
        executors[group_id] = MissionExecutor(package.parsed_tree.tree, services)

    ticks = 0
    terminal = {NodeStatus.SUCCESS, NodeStatus.FAILURE, NodeStatus.CANCELLED}
    while ticks < max_ticks:
        for group_id in ("GroupA", "GroupB"):
            executor = executors[group_id]
            if executor.status not in terminal:
                executor.tick()
        ticks += 1
        if all(executor.status in terminal for executor in executors.values()):
            break
        clock.advance(tick_seconds)
    else:
        for executor in executors.values():
            if executor.status not in terminal:
                executor.abort("JOINT_SIMULATION_MAX_TICKS")

    result = JointSimulationResult(
        group_a_status=executors["GroupA"].status,
        group_b_status=executors["GroupB"].status,
        ticks=ticks,
        group_a_commands=len(command_transports["GroupA"].history),
        group_b_commands=len(command_transports["GroupB"].history),
    )
    for executor in executors.values():
        executor.close()
    return result
