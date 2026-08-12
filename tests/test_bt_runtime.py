"""Focused tests for strict parsing and tick-based node semantics."""

from types import SimpleNamespace

import pytest

from uav_bt_runtime.bt import (
    ActionNode,
    ActionRegistration,
    BehaviorTreeParseError,
    BehaviorTreeParser,
    FallbackNode,
    NodeStatus,
    SequenceNode,
)
from uav_bt_runtime.clock import ManualClock
from uav_bt_runtime.mission_log import MemoryMissionLogger


class FakeHandler:
    def __init__(self, start_status=NodeStatus.RUNNING, ticks=None):
        self.start_status = start_status
        self.ticks = list(ticks or [])
        self.started = 0
        self.cancelled = 0

    def start(self, services):
        del services
        self.started += 1
        return self.start_status

    def tick(self, services):
        del services
        return self.ticks.pop(0) if self.ticks else NodeStatus.RUNNING

    def cancel(self, services):
        del services
        self.cancelled += 1


def services():
    clock = ManualClock()
    return SimpleNamespace(
        clock=clock,
        logger=MemoryMissionLogger(clock),
        mission=SimpleNamespace(mission_id="mission-1", group_id="GroupA"),
    )


def test_sequence_resumes_running_child_and_starts_action_once():
    env = services()
    first_handler = FakeHandler(ticks=[NodeStatus.SUCCESS])
    second_handler = FakeHandler(start_status=NodeStatus.SUCCESS)
    sequence = SequenceNode(
        "tree:0",
        [
            ActionNode("tree:0:0", "First", first_handler, timeout_s=10),
            ActionNode("tree:0:1", "Second", second_handler, timeout_s=10),
        ],
    )
    assert sequence.tick(env) == NodeStatus.RUNNING
    assert sequence.tick(env) == NodeStatus.SUCCESS
    assert first_handler.started == 1
    assert second_handler.started == 1


def test_fallback_runs_reserve_only_after_primary_failure():
    env = services()
    primary = FakeHandler(start_status=NodeStatus.FAILURE)
    reserve = FakeHandler(start_status=NodeStatus.SUCCESS)
    fallback = FallbackNode(
        "tree:0",
        [
            ActionNode("tree:0:0", "Primary", primary),
            ActionNode("tree:0:1", "Reserve", reserve),
        ],
    )
    assert fallback.tick(env) == NodeStatus.SUCCESS
    assert primary.started == 1
    assert reserve.started == 1


def test_action_timeout_cancels_running_handler():
    env = services()
    handler = FakeHandler()
    action = ActionNode("tree:0", "Wait", handler, timeout_s=2.0)
    assert action.tick(env) == NodeStatus.RUNNING
    env.clock.advance(2.0)
    assert action.tick(env) == NodeStatus.TIMEOUT
    assert handler.cancelled == 1


def test_parser_selects_main_tree_and_records_plan_reference(tmp_path):
    xml = tmp_path / "tree.xml"
    xml.write_text(
        """
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <Action ID="Move" plan_id="p1" timeout_s="10"/>
    </Sequence>
  </BehaviorTree>
</root>
""".strip(),
        encoding="utf-8",
    )
    parser = BehaviorTreeParser(
        {
            "Move": ActionRegistration(
                factory=lambda instance_id, attrs: FakeHandler(
                    start_status=NodeStatus.SUCCESS
                ),
                required_attributes=frozenset({"plan_id", "timeout_s"}),
            )
        }
    )
    parsed = parser.parse(xml)
    assert parsed.tree.tree_id == "Main"
    assert parsed.action_references[0].plan_id == "p1"


def test_parser_rejects_invalid_unselected_tree(tmp_path):
    xml = tmp_path / "tree.xml"
    xml.write_text(
        """
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main"><Action ID="Known"/></BehaviorTree>
  <BehaviorTree ID="Unused"><Unknown/></BehaviorTree>
</root>
""".strip(),
        encoding="utf-8",
    )
    parser = BehaviorTreeParser(
        {
            "Known": ActionRegistration(
                factory=lambda instance_id, attrs: FakeHandler(
                    start_status=NodeStatus.SUCCESS
                )
            )
        }
    )
    with pytest.raises(BehaviorTreeParseError, match="unsupported"):
        parser.parse(xml)


@pytest.mark.parametrize(
    "xml_text, expected",
    [
        (
            '<root main_tree_to_execute="Main"><BehaviorTree ID="Main">'
            '<Action ID="Unknown"/></BehaviorTree></root>',
            "unknown Action",
        ),
        (
            '<root main_tree_to_execute="Main"><BehaviorTree ID="Main">'
            '<Sequence strange="x"><Action ID="Known"/></Sequence>'
            '</BehaviorTree></root>',
            "unknown attributes",
        ),
        (
            '<root main_tree_to_execute="Missing"><BehaviorTree ID="Main">'
            '<Action ID="Known"/></BehaviorTree></root>',
            "unknown BehaviorTree",
        ),
    ],
)
def test_parser_fails_closed_for_unknown_or_invalid_xml(tmp_path, xml_text, expected):
    xml = tmp_path / "bad.xml"
    xml.write_text(xml_text, encoding="utf-8")
    parser = BehaviorTreeParser(
        {
            "Known": ActionRegistration(
                factory=lambda instance_id, attrs: FakeHandler(
                    start_status=NodeStatus.SUCCESS
                )
            )
        }
    )
    with pytest.raises(BehaviorTreeParseError, match=expected):
        parser.parse(xml)
