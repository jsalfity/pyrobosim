#!/usr/bin/env python3

"""
Behavior trees written in py_trees BehaviorTree XML (pyrobosim.behaviors.pytree_xml).

Structural tests use a stand-in robot; one test runs a tree in the ROSCon
workshop world.
"""

from __future__ import annotations

import time
from typing import Any

import py_trees
import pytest

from pyrobosim.behaviors import (
    NONE_SENTINEL,
    Check,
    RobotActionBehavior,
    blackboard_keys,
    build_pyrobosim_tree_from_xml,
)
from pyrobosim.core import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


def tree(body: str, main: bool = True) -> str:
    head = '<root main_tree_to_execute="Main">' if main else "<root>"
    return f'{head}<BehaviorTree ID="Main">{body}</BehaviorTree></root>'


class _StandIn:
    """Enough of a robot to build nodes; nothing here is executed."""


@pytest.fixture(autouse=True)
def clear_blackboard() -> None:
    py_trees.blackboard.Blackboard.clear()


def nodes(xml: str, **kwargs: Any) -> list[py_trees.behaviour.Behaviour]:
    return list(build_pyrobosim_tree_from_xml(xml, _StandIn(), **kwargs).root.iterate())


def test_skills_take_literal_and_blackboard_inputs() -> None:
    built = nodes(
        tree(
            '<Sequence name="s" memory="true">'
            '<navigate name="go" target_location="pantry"/>'
            '<pick name="take" object="{target}"/>'
            "</Sequence>"
        ),
        blackboard={"target": "bread"},
    )
    go, take = (n for n in built if isinstance(n, RobotActionBehavior))
    assert (go.action_type, go.params()) == ("navigate", {"target_location": "pantry"})
    assert (take.action_type, take.params()) == ("pick", {"object": "bread"})


def test_unwired_input_is_left_out() -> None:
    (place,) = (n for n in nodes(tree('<place name="drop"/>')) if isinstance(n, RobotActionBehavior))
    assert place.params() == {}


def test_written_keys_are_state_keys_plus_wired_outputs() -> None:
    built = build_pyrobosim_tree_from_xml(
        tree(
            '<Sequence name="s" memory="true">'
            '<detect name="look" object="bread" detected_categories="{seen}"/>'
            '<Check name="saw" key="seen" operator="contains" value="bread"/>'
            "</Sequence>"
        ),
        _StandIn(),
    )
    assert blackboard_keys(built) == {
        "/robot_location", "/held_category", "/objects_here", "/open_locations", "/seen",
    }


@pytest.mark.parametrize(
    "body",
    [
        '<grab name="g" object="bread"/>',  # not a skill
        '<pick name="p" item="bread"/>',  # not a parameter of pick
        '<place name="p" object="bread"/>',  # place takes no object
        '<ReactiveSequence name="r"><pick name="p" object="bread"/></ReactiveSequence>',  # BT.CPP only
        '<Check name="c" key="held_category" operator="=~" value="bread"/>',  # not an operator
        '<Retry name="r"><pick name="p" object="bread"/></Retry>',  # Retry needs num_failures
    ],
)
def test_parser_rejects(body: str) -> None:
    with pytest.raises(Exception):
        nodes(tree(body))


def test_single_tree_needs_no_main_tree_attribute() -> None:
    built = nodes(tree('<navigate name="go" target_location="pantry"/>', main=False))
    assert isinstance(built[0], RobotActionBehavior)


def check(key: str, operator: str, value: str) -> py_trees.common.Status:
    (node,) = nodes(tree(f'<Check name="c" key="{key}" operator="{operator}" value=\'{value}\'/>'))
    assert isinstance(node, Check)
    node.tick_once()
    return node.status


def test_check_fails_on_a_key_nothing_wrote() -> None:
    assert check("held_category", "==", NONE_SENTINEL) == py_trees.common.Status.FAILURE


def test_check_reads_json_values() -> None:
    writer = py_trees.blackboard.Client(name="writer")
    writer.register_key(key="robot_location", access=py_trees.common.Access.WRITE)
    writer.set("robot_location", "table")
    assert check("robot_location", "in", '["table", "table_tabletop"]') == py_trees.common.Status.SUCCESS
    assert check("robot_location", "==", "desk") == py_trees.common.Status.FAILURE


def test_tree_runs_in_world() -> None:
    world = WorldYamlLoader().from_file(get_data_folder() / "roscon_2024_workshop_world.yaml")
    try:
        built = build_pyrobosim_tree_from_xml(
            tree(
                '<Sequence name="fetch" memory="true">'
                '<navigate name="go" target_location="pantry"/>'
                '<detect name="look" object="{target}" detected_categories="{seen}"/>'
                '<Check name="saw" key="seen" operator="contains" value="bread"/>'
                '<pick name="take" object="{target}"/>'
                "</Sequence>"
            ),
            world.robots[0],
            realtime_factor=25.0,
            blackboard={"target": "bread"},
        )
        end = time.monotonic() + 60.0
        while built.root.status not in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
            assert time.monotonic() < end
            built.tick()
            time.sleep(0.05)
        storage = py_trees.blackboard.Blackboard.storage
        assert built.root.status == py_trees.common.Status.SUCCESS
        assert storage["/seen"] == ["bread"]
        assert storage["/held_category"] == "bread"
        assert storage["/robot_location"] == "pantry"
    finally:
        world.shutdown()
