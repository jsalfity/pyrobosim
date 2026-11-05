from __future__ import annotations

from pyrobosim.mcp.providers import PyRoboSimRootstocksProvider


def test_rootstocks_provider_loads_expected_templates() -> None:
    rootstocks = PyRoboSimRootstocksProvider().get_rootstocks()
    names = {rootstock["name"] for rootstock in rootstocks}

    assert {"detect_pick", "search_pick_selector", "search_pick_deliver"}.issubset(names)
    assert all("template" in rootstock for rootstock in rootstocks)


def test_every_template_passes_the_gate_once_filled() -> None:
    """Each template, slots filled and every optional node kept, is a tree the robot accepts."""
    import re

    from pyrobosim.mcp.validation import validate_bt_xml

    values = {
        "object_query": "bread",
        "source_location": "pantry",
        "primary_location": "pantry",
        "fallback_location": "fridge",
        "destination_location": "table",
    }
    vocabulary = {"locations": {"pantry", "fridge", "table"}, "objects": {"bread"}}
    for rootstock in PyRoboSimRootstocksProvider().get_rootstocks():
        template = rootstock["template"]
        assert set(re.findall(r"\$([a-z_]+)", template)) == set(rootstock["slots"]), rootstock["name"]
        for node in rootstock.get("optional_nodes", {}):
            assert f'name="{node}"' in template, (rootstock["name"], node)
        filled = re.sub(r"\$([a-z_]+)", lambda m: values[m.group(1)], template)
        xml = f'<root main_tree_to_execute="Main"><BehaviorTree ID="Main">{filled}</BehaviorTree></root>'
        assert validate_bt_xml(xml, vocabulary=vocabulary) == [], rootstock["name"]
