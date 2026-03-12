"""BT JSON schema (YAML-backed)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


_DATA_PATH = Path(__file__).resolve().parent / "data" / "bt_schema.yaml"


def get_bt_schema(restrict_control_flow: bool = False) -> dict[str, Any]:
    data = yaml.safe_load(_DATA_PATH.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        return {}

    if restrict_control_flow:
        # For sequence-only baseline: remove selector, parallel, decorator
        data = data.copy()
        data["node_types"] = ["sequence", "action", "condition"]
        # Remove non-sequence composite nodes from schema
        if "composite_nodes" in data:
            composite = data["composite_nodes"]
            data["composite_nodes"] = {
                "sequence": composite.get("sequence", {})
            }

    return data
