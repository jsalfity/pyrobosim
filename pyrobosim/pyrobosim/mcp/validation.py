"""Validation gate for PyRoboSim behavior trees in py_trees BehaviorTree XML.

Two layers, run in this order:

1. The contract: only the node tags this robot offers, an explicit memory on
   every Sequence and Selector, children where a node needs them, and, when a
   world vocabulary is available, skill inputs that name things in the world.
2. py_trees' own XML parser, building the tree with the runtime's node classes
   and no robot. A tree the robot would refuse to build is refused here, with
   the parser's message.

The second layer runs only when the first finds nothing, so a tree with an
unknown tag gets one clear issue rather than the same problem twice.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Any

from pyrobosim.behaviors import SKILLS, build_pyrobosim_tree_from_xml

COMPOSITES = ("Sequence", "Selector", "Fallback", "Parallel")
DECORATORS = ("Retry", "Inverter")
CONDITIONS = ("Check",)
#: Composites whose memory py_trees defaults to true when it is left out.
MEMORY_COMPOSITES = ("Sequence", "Selector", "Fallback")

#: World-vocabulary keys that hold names a skill input may use.
LOCATION_ENTITY_KEYS = ("rooms", "locations", "object_spawns")
OBJECT_ENTITY_KEYS = ("objects", "object_categories")
LOCATION_INPUTS = ("target_location",)
OBJECT_INPUTS = ("object",)


def allowed_tags(restrict_control_flow: bool = False) -> tuple[str, ...]:
    """Every node tag a tree may use."""
    control = ("Sequence",) if restrict_control_flow else COMPOSITES + DECORATORS
    return control + CONDITIONS + tuple(SKILLS)


def vocabulary_from_entities(entities: dict[str, Any]) -> dict[str, set[str]]:
    """Location and object names from a /world_entities response."""
    return {
        "locations": {name for key in LOCATION_ENTITY_KEYS for name in entities.get(key, [])},
        "objects": {name for key in OBJECT_ENTITY_KEYS for name in entities.get(key, [])},
    }


def _issue(code: str, message: str, path: str) -> dict[str, Any]:
    return {"code": code, "message": message, "path": path}


def _label(elem: ET.Element) -> str:
    return f"{elem.tag}({elem.get('name')})" if elem.get("name") else elem.tag


def _is_key(value: str) -> bool:
    return value.startswith("{") and value.endswith("}")


def _check_node(
    elem: ET.Element,
    path: str,
    allowed: tuple[str, ...],
    vocabulary: dict[str, set[str]] | None,
) -> list[dict[str, Any]]:
    here = f"{path}/{_label(elem)}"
    tag = elem.tag
    children = list(elem)
    if tag not in allowed:
        return [_issue(
            "unknown_node_type",
            f"'{tag}' is not a node this robot offers. Allowed: {', '.join(allowed)}.",
            here,
        )]

    issues: list[dict[str, Any]] = []
    if tag in MEMORY_COMPOSITES:
        memory = elem.get("memory")
        if memory is None:
            issues.append(_issue(
                "missing_memory",
                f'{tag} must say memory="true" or memory="false"; py_trees would assume true.',
                here,
            ))
        elif memory not in ("true", "false"):
            issues.append(_issue("invalid_memory", f'memory must be "true" or "false", not "{memory}".', here))
    if tag in COMPOSITES and not children:
        issues.append(_issue("empty_composite", f"{tag} needs at least one child.", here))
    if tag in DECORATORS and len(children) != 1:
        issues.append(_issue("decorator_children", f"{tag} needs exactly one child, not {len(children)}.", here))
    if tag in CONDITIONS or tag in SKILLS:
        if children:
            issues.append(_issue("leaf_with_children", f"{tag} cannot have children.", here))
    if tag in SKILLS and vocabulary is not None:
        for attr, kind in [(a, "locations") for a in LOCATION_INPUTS] + [(a, "objects") for a in OBJECT_INPUTS]:
            value = elem.get(attr)
            if value is not None and not _is_key(value) and value not in vocabulary[kind]:
                issues.append(_issue(
                    "unknown_vocabulary",
                    f"{attr}='{value}' is not in the world vocabulary ({kind}); see list_world_entities.",
                    here,
                ))
    for child in children:
        issues += _check_node(child, here, allowed, vocabulary)
    return issues


def _check_document(root: ET.Element) -> list[dict[str, Any]]:
    if root.tag != "root":
        return [_issue("invalid_document", f"The document element must be <root>, not <{root.tag}>.", "")]
    issues = [
        _issue("unsupported_element", f"<{child.tag}> is not supported here; only <BehaviorTree> elements may sit under <root>.", child.tag)
        for child in root
        if child.tag != "BehaviorTree"
    ]
    trees = root.findall("BehaviorTree")
    ids = [tree.get("ID") for tree in trees]
    main = root.get("main_tree_to_execute")
    if not trees:
        issues.append(_issue("missing_tree", "The document needs a <BehaviorTree ID=\"Main\"> element.", ""))
    elif main is None and len(trees) > 1:
        issues.append(_issue("missing_main_tree", "With several BehaviorTree elements, <root> must name one in main_tree_to_execute.", ""))
    elif main is not None and main not in ids:
        issues.append(_issue("missing_main_tree", f"main_tree_to_execute='{main}' names no BehaviorTree (have {ids}).", ""))
    for tree in trees:
        if len(tree) != 1:
            issues.append(_issue(
                "tree_root",
                f"BehaviorTree '{tree.get('ID')}' must contain exactly one node, not {len(tree)}.",
                str(tree.get("ID")),
            ))
    return issues


def validate_bt_xml(
    bt_xml: str,
    restrict_control_flow: bool = False,
    vocabulary: dict[str, set[str]] | None = None,
) -> list[dict[str, Any]]:
    """Issues with a behavior tree, or [] when the robot would accept it.

    Args:
        bt_xml: The tree, as an XML string.
        restrict_control_flow: Offer Sequence as the only control node.
        vocabulary: Location and object names (see vocabulary_from_entities),
            or None to skip the vocabulary check.
    """
    try:
        root = ET.fromstring(bt_xml)
    except ET.ParseError as exc:
        return [_issue("xml_parse_error", f"Not well-formed XML: {exc}", "")]

    issues = _check_document(root)
    allowed = allowed_tags(restrict_control_flow)
    for tree in root.findall("BehaviorTree"):
        for node in tree:
            issues += _check_node(node, str(tree.get("ID")), allowed, vocabulary)
    if issues:
        return issues

    try:
        build_pyrobosim_tree_from_xml(bt_xml, robot=None)
    except Exception as exc:  # the parser raises several types; every one is a rejection
        issues.append(_issue("py_trees_parse_error", f"{type(exc).__name__}: {exc}", ""))
    return issues
