"""Tick-based Sequence, Fallback, Condition, and Action nodes."""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum
from typing import Any, Callable, List, Optional, Protocol


class NodeStatus(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    CANCELLED = "CANCELLED"
    TIMEOUT = "TIMEOUT"


FAILURE_STATUSES = {
    NodeStatus.FAILURE,
    NodeStatus.CANCELLED,
    NodeStatus.TIMEOUT,
}


class ActionHandler(Protocol):
    """Non-blocking lifecycle implemented by semantic mission actions."""

    def start(self, services: Any) -> NodeStatus:
        """Start the action once."""

    def tick(self, services: Any) -> NodeStatus:
        """Poll progress without blocking."""

    def cancel(self, services: Any) -> None:
        """Cancel any lower-layer work owned by the action."""


class BaseNode(ABC):
    """Base node with transition logging and explicit halt semantics."""

    def __init__(self, instance_id: str) -> None:
        self.instance_id = instance_id
        self.status = NodeStatus.IDLE

    def _transition(self, services: Any, status: NodeStatus) -> NodeStatus:
        previous = self.status
        self.status = status
        if previous != status:
            services.logger.emit(
                "node_transition",
                mission_id=services.mission.mission_id,
                group_id=services.mission.group_id,
                node_id=self.instance_id,
                node_type=type(self).__name__,
                previous_status=previous.value,
                status=status.value,
            )
        return status

    @abstractmethod
    def tick(self, services: Any) -> NodeStatus:
        """Advance this node by one non-blocking tick."""

    def halt(self, services: Any, status: NodeStatus = NodeStatus.CANCELLED) -> None:
        self._transition(services, status)


class SequenceNode(BaseNode):
    """Memory sequence: resume the currently running child on the next tick."""

    def __init__(self, instance_id: str, children: List[BaseNode]) -> None:
        if not children:
            raise ValueError("Sequence requires at least one child")
        super().__init__(instance_id)
        self.children = children
        self.current_index = 0

    def tick(self, services: Any) -> NodeStatus:
        while self.current_index < len(self.children):
            child_status = self.children[self.current_index].tick(services)
            if child_status == NodeStatus.SUCCESS:
                self.current_index += 1
                continue
            if child_status == NodeStatus.RUNNING:
                return self._transition(services, NodeStatus.RUNNING)
            return self._transition(services, NodeStatus.FAILURE)
        return self._transition(services, NodeStatus.SUCCESS)

    def halt(self, services: Any, status: NodeStatus = NodeStatus.CANCELLED) -> None:
        if self.current_index < len(self.children):
            child = self.children[self.current_index]
            if child.status == NodeStatus.RUNNING:
                child.halt(services, status)
        super().halt(services, status)


class FallbackNode(BaseNode):
    """Memory fallback: try the next child only after the current one fails."""

    def __init__(self, instance_id: str, children: List[BaseNode]) -> None:
        if not children:
            raise ValueError("Fallback requires at least one child")
        super().__init__(instance_id)
        self.children = children
        self.current_index = 0

    def tick(self, services: Any) -> NodeStatus:
        while self.current_index < len(self.children):
            child_status = self.children[self.current_index].tick(services)
            if child_status == NodeStatus.SUCCESS:
                return self._transition(services, NodeStatus.SUCCESS)
            if child_status == NodeStatus.RUNNING:
                return self._transition(services, NodeStatus.RUNNING)
            self.current_index += 1
        return self._transition(services, NodeStatus.FAILURE)

    def halt(self, services: Any, status: NodeStatus = NodeStatus.CANCELLED) -> None:
        if self.current_index < len(self.children):
            child = self.children[self.current_index]
            if child.status == NodeStatus.RUNNING:
                child.halt(services, status)
        super().halt(services, status)


class ConditionNode(BaseNode):
    """Instantaneous check; waiting is intentionally implemented as an Action."""

    def __init__(
        self,
        instance_id: str,
        condition_id: str,
        check: Callable[[Any], bool],
    ) -> None:
        super().__init__(instance_id)
        self.condition_id = condition_id
        self.check = check

    def tick(self, services: Any) -> NodeStatus:
        try:
            result = bool(self.check(services))
        except Exception as exc:
            services.logger.emit(
                "condition_error",
                mission_id=services.mission.mission_id,
                group_id=services.mission.group_id,
                node_id=self.instance_id,
                condition_id=self.condition_id,
                error=str(exc),
            )
            result = False
        return self._transition(
            services, NodeStatus.SUCCESS if result else NodeStatus.FAILURE
        )


class ActionNode(BaseNode):
    """Non-blocking semantic action with task-level timeout and cancellation."""

    def __init__(
        self,
        instance_id: str,
        action_id: str,
        handler: ActionHandler,
        timeout_s: Optional[float] = None,
    ) -> None:
        super().__init__(instance_id)
        if timeout_s is not None and timeout_s <= 0:
            raise ValueError("Action timeout_s must be positive")
        self.action_id = action_id
        self.handler = handler
        self.timeout_s = timeout_s
        self.started_at: Optional[float] = None

    def tick(self, services: Any) -> NodeStatus:
        if self.status in {
            NodeStatus.SUCCESS,
            NodeStatus.FAILURE,
            NodeStatus.CANCELLED,
            NodeStatus.TIMEOUT,
        }:
            return self.status

        try:
            if self.status == NodeStatus.IDLE:
                self.started_at = services.clock.monotonic()
                result = self.handler.start(services)
            else:
                if (
                    self.timeout_s is not None
                    and self.started_at is not None
                    and services.clock.monotonic() - self.started_at >= self.timeout_s
                ):
                    self._cancel_safely(services)
                    return self._transition(services, NodeStatus.TIMEOUT)
                result = self.handler.tick(services)
        except Exception as exc:
            self._cancel_safely(services)
            services.logger.emit(
                "action_error",
                mission_id=services.mission.mission_id,
                group_id=services.mission.group_id,
                node_id=self.instance_id,
                action_id=self.action_id,
                error=str(exc),
            )
            result = NodeStatus.FAILURE

        if not isinstance(result, NodeStatus):
            raise TypeError(f"action {self.action_id!r} returned invalid status {result!r}")
        return self._transition(services, result)

    def halt(self, services: Any, status: NodeStatus = NodeStatus.CANCELLED) -> None:
        if self.status == NodeStatus.RUNNING:
            self._cancel_safely(services)
        super().halt(services, status)

    def _cancel_safely(self, services: Any) -> None:
        try:
            self.handler.cancel(services)
        except Exception as exc:
            services.logger.emit(
                "action_cancel_error",
                mission_id=services.mission.mission_id,
                group_id=services.mission.group_id,
                node_id=self.instance_id,
                action_id=self.action_id,
                error=str(exc),
            )


class BehaviorTree:
    """Selected BehaviorTree and its single executable root node."""

    def __init__(self, tree_id: str, root: BaseNode) -> None:
        self.tree_id = tree_id
        self.root = root

    @property
    def status(self) -> NodeStatus:
        return self.root.status

    def tick(self, services: Any) -> NodeStatus:
        return self.root.tick(services)

    def halt(self, services: Any, status: NodeStatus = NodeStatus.CANCELLED) -> None:
        self.root.halt(services, status)
