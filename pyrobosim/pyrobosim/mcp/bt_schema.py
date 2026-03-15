"""BT schema utilities for PyRoboSim MCP server."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def get_bt_schema() -> dict[str, Any]:
    """
    Load and return the BT schema from data/bt_schema.yaml.

    Returns:
        BT schema dict with node_types, action_node, condition_node, etc.
    """
    schema_path = Path(__file__).parent / "data" / "bt_schema.yaml"
    with open(schema_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)
