"""Mission services, event inbox, and one-Group behavior-tree executor."""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from typing import Dict, Optional, Set, Tuple

from .bt import BehaviorTree, NodeStatus
from .clock import Clock, SystemClock
from .mission_log import MissionLogger, NullMissionLogger
from .models import CommandEnvelope, CoordinationEvent, MissionContext
from .transport import CommandTransport, CoordinationTransport


class EventStore:
    """Deduplicated stage-event store keyed by source Group and event type."""

    def __init__(self) -> None:
        self._events: Dict[Tuple[str, str], CoordinationEvent] = {}
        self._seen_ids: Set[str] = set()
        self._last_sequence: Dict[str, int] = {}

    def add(self, event: CoordinationEvent) -> bool:
        if event.event_id in self._seen_ids:
            return False
        last_sequence = self._last_sequence.get(event.source_group, -1)
        if event.sequence_number <= last_sequence:
            return False
        self._seen_ids.add(event.event_id)
        self._last_sequence[event.source_group] = event.sequence_number
        self._events[(event.source_group, event.event_type)] = event
        return True

    def has(self, event_type: str, source_group: Optional[str] = None) -> bool:
        if source_group is not None:
            return (source_group, event_type) in self._events
        return any(key[1] == event_type for key in self._events)


@dataclass
class RuntimeServices:
    """Runtime dependencies injected into all behavior-tree nodes and actions."""

    mission: MissionContext
    command_transport: CommandTransport
    coordination_transport: CoordinationTransport
    clock: Clock = field(default_factory=SystemClock)
    logger: MissionLogger = field(default_factory=NullMissionLogger)
    events: EventStore = field(default_factory=EventStore)
    _sequence_number: int = 0

    def next_sequence(self) -> int:
        sequence = self._sequence_number
        self._sequence_number += 1
        return sequence

    def ingest_events(self) -> None:
        for event in self.coordination_transport.poll_events():
            if event.mission_id != self.mission.mission_id:
                self.logger.emit(
                    "event_rejected",
                    mission_id=self.mission.mission_id,
                    group_id=self.mission.group_id,
                    reason="MISSION_ID_MISMATCH",
                    event_id=event.event_id,
                )
                continue
            if event.source_group == self.mission.group_id:
                continue
            if self.events.add(event):
                self.logger.emit(
                    "coordination_event_received",
                    mission_id=self.mission.mission_id,
                    group_id=self.mission.group_id,
                    source_group=event.source_group,
                    event_id=event.event_id,
                    event_type=event.event_type,
                )

    def publish_event(self, event_type: str) -> CoordinationEvent:
        event = CoordinationEvent(
            mission_id=self.mission.mission_id,
            event_id=f"event-{uuid.uuid4().hex}",
            source_group=self.mission.group_id,
            event_type=event_type,
            sequence_number=self.next_sequence(),
            timestamp=self.clock.time(),
        )
        self.coordination_transport.publish(event)
        self.logger.emit(
            "coordination_event_published",
            mission_id=self.mission.mission_id,
            group_id=self.mission.group_id,
            event_id=event.event_id,
            event_type=event.event_type,
        )
        return event


class MissionExecutor:
    """Run one Group's selected tree and enforce mission-level abort behavior."""

    def __init__(self, tree: BehaviorTree, services: RuntimeServices) -> None:
        self.tree = tree
        self.services = services
        self.status = NodeStatus.IDLE
        self.abort_published = False
        self.finished_logged = False

    def tick(self) -> NodeStatus:
        if self.status in {
            NodeStatus.SUCCESS,
            NodeStatus.FAILURE,
            NodeStatus.CANCELLED,
        }:
            return self.status

        try:
            self.services.ingest_events()
        except Exception as exc:
            self.services.logger.emit(
                "coordination_poll_error",
                mission_id=self.services.mission.mission_id,
                group_id=self.services.mission.group_id,
                error=str(exc),
            )
            return self.abort("COORDINATION_POLL_ERROR", publish=True)
        if self.services.events.has("MISSION_ABORT"):
            return self.abort("PEER_MISSION_ABORT", publish=False)

        self.status = self.tree.tick(self.services)
        if self.status == NodeStatus.SUCCESS:
            if not self.finished_logged:
                self.services.logger.emit(
                    "mission_finished",
                    mission_id=self.services.mission.mission_id,
                    group_id=self.services.mission.group_id,
                    status=NodeStatus.SUCCESS.value,
                )
                self.finished_logged = True
            return self.status

        if self.status in {NodeStatus.FAILURE, NodeStatus.TIMEOUT}:
            return self.abort(f"TREE_{self.status.value}", publish=True)
        return self.status

    def abort(self, reason: str, publish: bool = True) -> NodeStatus:
        if self.status == NodeStatus.SUCCESS:
            return self.status
        if self.tree.status == NodeStatus.RUNNING:
            self.tree.halt(self.services, NodeStatus.CANCELLED)
        if publish and not self.abort_published:
            self._publish_abort_safely()
            self.abort_published = True
        self._request_safe_hold(reason)
        self.status = NodeStatus.FAILURE
        if not self.finished_logged:
            self.services.logger.emit(
                "mission_finished",
                mission_id=self.services.mission.mission_id,
                group_id=self.services.mission.group_id,
                status=NodeStatus.FAILURE.value,
                reason=reason,
            )
            self.finished_logged = True
        return self.status

    def cancel(self, reason: str = "OPERATOR_CANCELLED") -> NodeStatus:
        if self.status == NodeStatus.SUCCESS:
            return self.status
        if self.tree.status == NodeStatus.RUNNING:
            self.tree.halt(self.services, NodeStatus.CANCELLED)
        if not self.abort_published:
            self._publish_abort_safely()
            self.abort_published = True
        self._request_safe_hold(reason)
        self.status = NodeStatus.CANCELLED
        if not self.finished_logged:
            self.services.logger.emit(
                "mission_finished",
                mission_id=self.services.mission.mission_id,
                group_id=self.services.mission.group_id,
                status=NodeStatus.CANCELLED.value,
                reason=reason,
            )
            self.finished_logged = True
        return self.status

    def _request_safe_hold(self, reason: str) -> None:
        mission = self.services.mission
        emergency_hold = getattr(self.services.command_transport, "emergency_hold", None)
        if callable(emergency_hold):
            try:
                emergency_hold(mission.mission_id)
            except Exception as exc:
                self.services.logger.emit(
                    "safe_hold_send_error",
                    mission_id=mission.mission_id,
                    group_id=mission.group_id,
                    error=str(exc),
                )
            else:
                self.services.logger.emit(
                    "emergency_hold_requested",
                    mission_id=mission.mission_id,
                    group_id=mission.group_id,
                    reason=reason,
                )
            return

        effective_ids = [
            uav_id
            for uav_id in mission.roster.active_ids + mission.roster.reserve_ids
            if uav_id not in mission.roster.failed_ids
        ]
        for uav_id in effective_ids:
            issued_at = self.services.clock.time()
            command = CommandEnvelope(
                mission_id=mission.mission_id,
                action_instance_id="mission-abort",
                command_id=f"safe-hold-{uuid.uuid4().hex}",
                group_id=mission.group_id,
                robot_id=uav_id,
                command_type="SAFE_HOLD",
                payload={"reason": reason},
                timeout_s=5.0,
                issued_at=issued_at,
                expires_at=issued_at + 5.0,
                sequence_number=self.services.next_sequence(),
            )
            try:
                self.services.command_transport.send(command)
            except Exception as exc:
                self.services.logger.emit(
                    "safe_hold_send_error",
                    mission_id=command.mission_id,
                    group_id=command.group_id,
                    robot_id=command.robot_id,
                    error=str(exc),
                )
            else:
                self.services.logger.emit(
                    "command_sent",
                    mission_id=command.mission_id,
                    group_id=command.group_id,
                    action_instance_id=command.action_instance_id,
                    command_id=command.command_id,
                    robot_id=command.robot_id,
                    command_type=command.command_type,
                    stage="ABORT",
                )

    def _publish_abort_safely(self) -> None:
        try:
            self.services.publish_event("MISSION_ABORT")
        except Exception as exc:
            self.services.logger.emit(
                "mission_abort_publish_error",
                mission_id=self.services.mission.mission_id,
                group_id=self.services.mission.group_id,
                error=str(exc),
            )

    def run(self, tick_hz: float = 10.0, max_ticks: Optional[int] = None) -> NodeStatus:
        if tick_hz <= 0:
            raise ValueError("tick_hz must be positive")
        period = 1.0 / tick_hz
        ticks = 0
        while self.status not in {
            NodeStatus.SUCCESS,
            NodeStatus.FAILURE,
            NodeStatus.CANCELLED,
        }:
            self.tick()
            ticks += 1
            if max_ticks is not None and ticks >= max_ticks:
                return self.abort("MAX_TICKS_EXCEEDED")
            if self.status == NodeStatus.RUNNING:
                time.sleep(period)
        return self.status

    def close(self) -> None:
        self.services.command_transport.close()
        self.services.coordination_transport.close()
        self.services.logger.close()
