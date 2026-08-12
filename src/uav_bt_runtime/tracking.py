"""Correlation and monotonic aggregation of per-UAV command feedback."""

from __future__ import annotations

from enum import Enum
from typing import Dict, Iterable, List, Optional

from .models import CommandEnvelope, CommandFeedback, CommandStatus


class BatchResult(str, Enum):
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


TERMINAL_STATUSES = {
    CommandStatus.SUCCESS,
    CommandStatus.FAILURE,
    CommandStatus.REJECTED,
    CommandStatus.CANCELLED,
    CommandStatus.TIMEOUT,
}


class CommandBatchTracker:
    """Track one fan-out batch and ignore stale, duplicate, or mismatched feedback."""

    def __init__(self, commands: Iterable[CommandEnvelope]) -> None:
        command_list = list(commands)
        self.commands: Dict[str, CommandEnvelope] = {
            command.command_id: command for command in command_list
        }
        if len(self.commands) != len(command_list):
            raise ValueError("command batch contains duplicate command IDs")
        if not self.commands:
            raise ValueError("command batch cannot be empty")
        self.statuses: Dict[str, Optional[CommandStatus]] = {
            command_id: None for command_id in self.commands
        }
        self.feedback: Dict[str, CommandFeedback] = {}

    def apply(self, feedback: CommandFeedback) -> bool:
        command = self.commands.get(feedback.command_id)
        if command is None:
            return False
        if (
            feedback.mission_id != command.mission_id
            or feedback.action_instance_id != command.action_instance_id
            or feedback.group_id != command.group_id
            or feedback.robot_id != command.robot_id
        ):
            return False

        current = self.statuses[feedback.command_id]
        if current in TERMINAL_STATUSES:
            return False
        if current == feedback.status:
            return False

        self.statuses[feedback.command_id] = feedback.status
        self.feedback[feedback.command_id] = feedback
        return True

    @property
    def result(self) -> BatchResult:
        statuses = list(self.statuses.values())
        failure_statuses = TERMINAL_STATUSES - {CommandStatus.SUCCESS}
        if any(status in failure_statuses for status in statuses):
            return BatchResult.FAILURE
        if statuses and all(status == CommandStatus.SUCCESS for status in statuses):
            return BatchResult.SUCCESS
        return BatchResult.RUNNING

    @property
    def active_command_ids(self) -> List[str]:
        return [
            command_id
            for command_id, status in self.statuses.items()
            if status not in TERMINAL_STATUSES
        ]
