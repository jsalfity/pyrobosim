#!/usr/bin/env python3

"""The validation gate for behavior trees in py_trees XML (pyrobosim.mcp.validation)."""

from __future__ import annotations

import pytest

from pyrobosim.mcp.validation import validate_bt_xml

VOCABULARY = {
    "locations": {"pantry", "fridge", "table", "table_tabletop", "kitchen"},
    "objects": {"bread", "soda", "bread0"},
}


def doc(body: str, main: str | None = "Main") -> str:
    head = f'<root main_tree_to_execute="{main}">' if main else "<root>"
    return f'{head}<BehaviorTree ID="Main">{body}</BehaviorTree></root>'


GOOD = (
    '<Sequence name="s" memory="true">'
    '<navigate name="n" target_location="pantry"/>'
    '<pick name="p" object="bread"/>'
    "</Sequence>"
)


def codes(xml: str, **kwargs: object) -> list[str]:
    return [issue["code"] for issue in validate_bt_xml(xml, **kwargs)]


def test_valid_tree_passes() -> None:
    assert codes(doc(GOOD), vocabulary=VOCABULARY) == []


def test_single_tree_needs_no_main_tree_attribute() -> None:
    assert codes(doc(GOOD, main=None)) == []


def test_blackboard_inputs_skip_the_vocabulary_check() -> None:
    body = '<Sequence name="s" memory="true"><pick name="p" object="{target}"/></Sequence>'
    assert codes(doc(body), vocabulary=VOCABULARY) == []


@pytest.mark.parametrize(
    "xml, code",
    [
        (doc(GOOD)[:-20], "xml_parse_error"),
        ('<tree><BehaviorTree ID="Main">' + GOOD + "</BehaviorTree></tree>", "invalid_document"),
        (doc(GOOD, main="Other"), "missing_main_tree"),
        (doc('<grab name="g" object="bread"/>'), "unknown_node_type"),
        (doc('<ReactiveSequence name="r"><pick name="p" object="bread"/></ReactiveSequence>'), "unknown_node_type"),
        (doc('<Timeout name="t" duration="5"><pick name="p" object="bread"/></Timeout>'), "unknown_node_type"),
        (doc('<Sequence name="s"><pick name="p" object="bread"/></Sequence>'), "missing_memory"),
        (doc('<Sequence name="s" memory="yes"><pick name="p" object="bread"/></Sequence>'), "invalid_memory"),
        (doc('<Sequence name="s" memory="true"></Sequence>'), "empty_composite"),
        (doc('<Retry name="r" num_failures="2"><pick name="a" object="bread"/><pick name="b" object="soda"/></Retry>'), "decorator_children"),
        (doc('<pick name="p" object="bread"><place name="q"/></pick>'), "leaf_with_children"),
        # Left to py_trees' own parser:
        (doc('<Sequence name="s" memory="true"><pick name="p" item="bread"/></Sequence>'), "py_trees_parse_error"),
        (doc('<Sequence name="s" memory="true"><place name="p" object="bread"/></Sequence>'), "py_trees_parse_error"),
        (doc('<Sequence memory="true"><pick name="p" object="bread"/><Sequence memory="true"><pick name="p" object="soda"/></Sequence></Sequence>'), "py_trees_parse_error"),
        (doc('<Retry name="r"><pick name="a" object="bread"/></Retry>'), "py_trees_parse_error"),
        (doc('<Sequence name="s" memory="true"><pick name="p" object="{target"/></Sequence>'), "py_trees_parse_error"),
        (doc('<Check name="c" key="held_category" operator="=~" value="bread"/>'), "py_trees_parse_error"),
    ],
)
def test_rejected(xml: str, code: str) -> None:
    assert code in codes(xml)


def test_vocabulary() -> None:
    body = '<Sequence name="s" memory="true"><navigate name="n" target_location="kitchenn"/><pick name="p" object="toast"/></Sequence>'
    assert codes(doc(body), vocabulary=VOCABULARY) == ["unknown_vocabulary", "unknown_vocabulary"]
    assert codes(doc(body)) == []  # no vocabulary, no check


def test_sequence_only_configuration() -> None:
    body = '<Selector name="s" memory="true"><pick name="p" object="bread"/></Selector>'
    assert codes(doc(body)) == []
    assert codes(doc(body), restrict_control_flow=True) == ["unknown_node_type"]
    assert codes(doc(GOOD), restrict_control_flow=True) == []
