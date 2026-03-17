from __future__ import annotations

from pyrobosim.mcp.providers import PyRoboSimRootstocksProvider


def test_rootstocks_provider_loads_expected_templates() -> None:
    rootstocks = PyRoboSimRootstocksProvider().get_rootstocks()
    names = {rootstock["name"] for rootstock in rootstocks}

    assert {"detect_pick", "search_pick_selector", "search_pick_deliver"}.issubset(names)
    assert all("template" in rootstock for rootstock in rootstocks)
