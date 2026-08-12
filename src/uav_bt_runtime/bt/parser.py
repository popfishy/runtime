"""Strict XML parser and registry-based behavior-tree node factory."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, FrozenSet, List, Mapping, Optional, Union

from .nodes import (
    ActionHandler,
    ActionNode,
    BaseNode,
    BehaviorTree,
    ConditionNode,
    FallbackNode,
    SequenceNode,
)


class BehaviorTreeParseError(ValueError):
    """Raised when XML syntax or behavior-tree semantics are invalid."""


ActionFactory = Callable[[str, Mapping[str, str]], ActionHandler]
ConditionFactory = Callable[[object], bool]


@dataclass(frozen=True)
class ActionRegistration:
    factory: ActionFactory
    required_attributes: FrozenSet[str] = frozenset()
    optional_attributes: FrozenSet[str] = frozenset()


@dataclass(frozen=True)
class ActionReference:
    instance_id: str
    action_id: str
    plan_id: Optional[str]


@dataclass(frozen=True)
class ParsedBehaviorTree:
    tree: BehaviorTree
    action_references: List[ActionReference]


class BehaviorTreeParser:
    """Parse only the minimal v1 node vocabulary and reject everything else."""

    def __init__(
        self,
        actions: Mapping[str, ActionRegistration],
        conditions: Optional[Mapping[str, ConditionFactory]] = None,
    ) -> None:
        self.actions = dict(actions)
        self.conditions = dict(conditions or {})
        self._references: List[ActionReference] = []

    def parse(self, path: Union[str, Path]) -> ParsedBehaviorTree:
        xml_path = Path(path)
        try:
            document = ET.parse(xml_path)
        except (OSError, ET.ParseError) as exc:
            raise BehaviorTreeParseError(f"cannot parse behavior tree {xml_path}: {exc}") from exc

        root = document.getroot()
        self._references = []
        if root.tag != "root":
            raise BehaviorTreeParseError("top-level XML tag must be <root>")
        self._reject_text(root)
        self._validate_attributes(root, {"main_tree_to_execute"}, set())
        selected_id = root.attrib["main_tree_to_execute"]

        behavior_trees = [child for child in root if child.tag == "BehaviorTree"]
        if len(behavior_trees) != len(list(root)):
            raise BehaviorTreeParseError("<root> may contain only <BehaviorTree> children")
        for element in behavior_trees:
            self._validate_attributes(element, {"ID"}, set())
            self._reject_text(element)
        ids = [element.attrib["ID"] for element in behavior_trees]
        if len(ids) != len(set(ids)):
            raise BehaviorTreeParseError("BehaviorTree IDs must be unique")
        if selected_id not in ids:
            raise BehaviorTreeParseError(
                f"main_tree_to_execute references unknown BehaviorTree {selected_id!r}"
            )

        parsed_nodes: Dict[str, BaseNode] = {}
        references_by_tree: Dict[str, List[ActionReference]] = {}
        for element in behavior_trees:
            tree_id = element.attrib["ID"]
            children = list(element)
            if len(children) != 1:
                raise BehaviorTreeParseError(
                    f"BehaviorTree {tree_id!r} must contain exactly one root node"
                )
            self._references = []
            parsed_nodes[tree_id] = self._parse_node(children[0], f"{tree_id}:0")
            references_by_tree[tree_id] = list(self._references)

        return ParsedBehaviorTree(
            tree=BehaviorTree(selected_id, parsed_nodes[selected_id]),
            action_references=references_by_tree[selected_id],
        )

    def _parse_node(self, element: ET.Element, instance_id: str) -> BaseNode:
        self._reject_text(element)
        if element.tag in {"Sequence", "Fallback"}:
            self._validate_attributes(element, set(), {"name"})
            children = [
                self._parse_node(child, f"{instance_id}:{index}")
                for index, child in enumerate(element)
            ]
            if not children:
                raise BehaviorTreeParseError(f"<{element.tag}> requires at least one child")
            if element.tag == "Sequence":
                return SequenceNode(instance_id, children)
            return FallbackNode(instance_id, children)

        if element.tag == "Action":
            if list(element):
                raise BehaviorTreeParseError("<Action> cannot contain child nodes")
            action_id = element.attrib.get("ID")
            if not action_id or action_id not in self.actions:
                raise BehaviorTreeParseError(f"unknown Action ID {action_id!r}")
            registration = self.actions[action_id]
            required = {"ID"} | set(registration.required_attributes)
            optional = {"timeout_s"} | set(registration.optional_attributes)
            self._validate_attributes(element, required, optional)
            timeout_s = self._parse_timeout(element.attrib.get("timeout_s"))
            handler = registration.factory(instance_id, dict(element.attrib))
            self._references.append(
                ActionReference(
                    instance_id=instance_id,
                    action_id=action_id,
                    plan_id=element.attrib.get("plan_id"),
                )
            )
            return ActionNode(instance_id, action_id, handler, timeout_s)

        if element.tag == "Condition":
            if list(element):
                raise BehaviorTreeParseError("<Condition> cannot contain child nodes")
            self._validate_attributes(element, {"ID"}, set())
            condition_id = element.attrib["ID"]
            check = self.conditions.get(condition_id)
            if check is None:
                raise BehaviorTreeParseError(f"unknown Condition ID {condition_id!r}")
            return ConditionNode(instance_id, condition_id, check)

        raise BehaviorTreeParseError(f"unsupported behavior-tree node <{element.tag}>")

    @staticmethod
    def _validate_attributes(
        element: ET.Element,
        required: set,
        optional: set,
    ) -> None:
        attributes = set(element.attrib)
        missing = required - attributes
        unknown = attributes - required - optional
        if missing:
            raise BehaviorTreeParseError(
                f"<{element.tag}> missing required attributes: {sorted(missing)}"
            )
        if unknown:
            raise BehaviorTreeParseError(
                f"<{element.tag}> has unknown attributes: {sorted(unknown)}"
            )

    @staticmethod
    def _parse_timeout(value: Optional[str]) -> Optional[float]:
        if value is None:
            return None
        try:
            timeout = float(value)
        except ValueError as exc:
            raise BehaviorTreeParseError("Action timeout_s must be numeric") from exc
        if timeout <= 0:
            raise BehaviorTreeParseError("Action timeout_s must be positive")
        return timeout

    @staticmethod
    def _reject_text(element: ET.Element) -> None:
        if element.text and element.text.strip():
            raise BehaviorTreeParseError(f"<{element.tag}> cannot contain text content")
