"""Transport protocols and deterministic in-memory implementations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Protocol, Sequence

from .clock import Clock
from .models import CommandEnvelope, CommandFeedback, CommandStatus, CoordinationEvent


class CommandTransport(Protocol):
    """Per-Group command/feedback boundary to a future execution layer."""

    def send(self, command: CommandEnvelope) -> None:
        """Submit one per-UAV command."""

    def cancel(self, command_id: str) -> None:
        """Request cancellation of one command."""

    def poll_feedback(self) -> List[CommandFeedback]:
        """Return feedback received since the previous poll."""

    def close(self) -> None:
        """Release transport resources."""


class CoordinationTransport(Protocol):
    """Task-stage event boundary between Group executors."""

    def publish(self, event: CoordinationEvent) -> None:
        """Publish one stage event."""

    def poll_events(self) -> List[CoordinationEvent]:
        """Return peer events received since the previous poll."""

    def close(self) -> None:
        """Release transport resources."""


@dataclass(frozen=True)
class ScriptedOutcome:
    """Terminal result returned by the scripted command backend."""

    status: CommandStatus = CommandStatus.SUCCESS
    delay_polls: int = 1
    error_code: Optional[str] = None
    error_message: Optional[str] = None

    def __post_init__(self) -> None:
        if self.delay_polls < 0:
            raise ValueError("delay_polls cannot be negative")
        if self.status in {CommandStatus.ACCEPTED, CommandStatus.RUNNING}:
            raise ValueError("a scripted outcome must be terminal")


@dataclass
class ScriptRule:
    """Match commands by semantic fields and override their terminal result."""

    outcome: ScriptedOutcome
    command_type: Optional[str] = None
    robot_id: Optional[str] = None
    plan_id: Optional[str] = None
    max_matches: Optional[int] = None
    matches: int = 0

    def applies(self, command: CommandEnvelope) -> bool:
        if self.max_matches is not None and self.matches >= self.max_matches:
            return False
        if self.command_type is not None and command.command_type != self.command_type:
            return False
        if self.robot_id is not None and command.robot_id != self.robot_id:
            return False
        if self.plan_id is not None and command.payload.get("_plan_id") != self.plan_id:
            return False
        return True


class ScriptedBackend:
    """Deterministic outcome selector for simulated lower-layer commands."""

    def __init__(
        self,
        rules: Optional[Sequence[ScriptRule]] = None,
        default: Optional[ScriptedOutcome] = None,
    ) -> None:
        self.rules = list(rules or [])
        self.default = default or ScriptedOutcome()

    def outcome_for(self, command: CommandEnvelope) -> ScriptedOutcome:
        for rule in self.rules:
            if rule.applies(command):
                rule.matches += 1
                return rule.outcome
        return self.default


@dataclass
class _CommandRecord:
    command: CommandEnvelope
    outcome: ScriptedOutcome
    remaining_polls: int
    running_emitted: bool = False
    terminal: bool = False


class InMemoryCommandTransport:
    """Poll-driven command transport used by tests and full joint simulation."""

    def __init__(
        self,
        group_id: str,
        clock: Clock,
        backend: Optional[ScriptedBackend] = None,
    ) -> None:
        self.group_id = group_id
        self.clock = clock
        self.backend = backend or ScriptedBackend()
        self._records: Dict[str, _CommandRecord] = {}
        self._pending: List[CommandFeedback] = []
        self.history: List[CommandEnvelope] = []
        self.closed = False

    def _feedback(
        self,
        command: CommandEnvelope,
        status: CommandStatus,
        error_code: Optional[str] = None,
        error_message: Optional[str] = None,
    ) -> CommandFeedback:
        return CommandFeedback(
            mission_id=command.mission_id,
            action_instance_id=command.action_instance_id,
            command_id=command.command_id,
            group_id=command.group_id,
            robot_id=command.robot_id,
            status=status,
            error_code=error_code,
            error_message=error_message,
            timestamp=self.clock.time(),
        )

    def send(self, command: CommandEnvelope) -> None:
        if self.closed:
            raise RuntimeError("command transport is closed")
        if command.group_id != self.group_id:
            raise ValueError(
                f"transport for {self.group_id!r} cannot send command for {command.group_id!r}"
            )
        if command.command_id in self._records:
            raise ValueError(f"duplicate command_id {command.command_id!r}")
        outcome = self.backend.outcome_for(command)
        self._records[command.command_id] = _CommandRecord(
            command=command,
            outcome=outcome,
            remaining_polls=outcome.delay_polls,
        )
        self.history.append(command)
        self._pending.append(self._feedback(command, CommandStatus.ACCEPTED))

    def cancel(self, command_id: str) -> None:
        record = self._records.get(command_id)
        if record is None or record.terminal:
            return
        record.terminal = True
        self._pending.append(self._feedback(record.command, CommandStatus.CANCELLED))

    def poll_feedback(self) -> List[CommandFeedback]:
        feedback = list(self._pending)
        self._pending.clear()

        for record in self._records.values():
            if record.terminal:
                continue
            if self.clock.time() >= record.command.expires_at:
                record.terminal = True
                feedback.append(self._feedback(record.command, CommandStatus.TIMEOUT))
                continue
            if not record.running_emitted:
                record.running_emitted = True
                feedback.append(self._feedback(record.command, CommandStatus.RUNNING))
                continue
            if record.remaining_polls > 0:
                record.remaining_polls -= 1
                continue

            record.terminal = True
            outcome = record.outcome
            feedback.append(
                self._feedback(
                    record.command,
                    outcome.status,
                    outcome.error_code,
                    outcome.error_message,
                )
            )
        return feedback

    def inject_feedback(self, feedback: CommandFeedback) -> None:
        """Inject duplicates or out-of-order feedback in focused tests."""

        self._pending.append(feedback)

    def close(self) -> None:
        self.closed = True


class InMemoryCoordinationBus:
    """Shared fan-out bus; each Group receives only peer events."""

    def __init__(self, drop_event_types: Optional[Sequence[str]] = None) -> None:
        self._queues: Dict[str, List[CoordinationEvent]] = {}
        self.drop_event_types = set(drop_event_types or [])

    def register(self, group_id: str) -> None:
        self._queues.setdefault(group_id, [])

    def publish(self, event: CoordinationEvent) -> None:
        if event.event_type in self.drop_event_types:
            return
        for group_id, queue in self._queues.items():
            if group_id != event.source_group:
                queue.append(event)

    def poll(self, group_id: str) -> List[CoordinationEvent]:
        queue = self._queues.setdefault(group_id, [])
        events = list(queue)
        queue.clear()
        return events


class InMemoryCoordinationTransport:
    """One Group endpoint connected to an InMemoryCoordinationBus."""

    def __init__(self, group_id: str, bus: InMemoryCoordinationBus) -> None:
        self.group_id = group_id
        self.bus = bus
        self.bus.register(group_id)
        self.closed = False

    def publish(self, event: CoordinationEvent) -> None:
        if self.closed:
            raise RuntimeError("coordination transport is closed")
        if event.source_group != self.group_id:
            raise ValueError("a Group cannot publish another Group's event")
        self.bus.publish(event)

    def poll_events(self) -> List[CoordinationEvent]:
        return self.bus.poll(self.group_id)

    def close(self) -> None:
        self.closed = True
