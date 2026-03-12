"""PyRoboSim-specific MCP providers."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from bt_mcp.providers import BTSchemaProvider, SkillProvider, WorldEntitiesProvider

_DATA_DIR = Path(__file__).resolve().parent / "data"


class PyRoboSimSkillProvider(SkillProvider):
    """Provides PyRoboSim skill schemas from YAML."""

    def __init__(self, skills_file: Path | None = None):
        """
        Initialize skill provider.

        Args:
            skills_file: Path to skills.yaml (defaults to data/skills.yaml)
        """
        self.skills_file = skills_file or (_DATA_DIR / "skills.yaml")

    def get_skills(self) -> list[dict[str, Any]]:
        """Load and return skill schemas from YAML."""
        data = yaml.safe_load(self.skills_file.read_text(encoding="utf-8"))
        skills = data.get("skills", []) if isinstance(data, dict) else []

        # Convert PyRoboSim skill format to bt-mcp-server format
        formatted_skills = []
        for skill in skills:
            formatted = {
                "name": skill["name"],
                "description": skill.get("description", ""),
                "parameters": skill.get("params", {}),
            }
            if "outputs" in skill:
                formatted["outputs"] = skill["outputs"]
            formatted_skills.append(formatted)

        return formatted_skills


class PyRoboSimBTSchemaProvider(BTSchemaProvider):
    """Provides PyRoboSim BT JSON schema from YAML."""

    def __init__(self, schema_file: Path | None = None):
        """
        Initialize BT schema provider.

        Args:
            schema_file: Path to bt_schema.yaml (defaults to data/bt_schema.yaml)
        """
        self.schema_file = schema_file or (_DATA_DIR / "bt_schema.yaml")

    def get_schema(self, restrict_control_flow: bool = False) -> dict[str, Any]:
        """
        Load and return BT schema from YAML.

        Args:
            restrict_control_flow: If True, restrict to sequence-only (removes selector/parallel/decorator)

        Returns:
            BT JSON schema specification
        """
        data = yaml.safe_load(self.schema_file.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            return {}

        if restrict_control_flow:
            # For sequence-only baseline: remove selector, parallel, decorator
            data = data.copy()
            data["node_types"] = ["sequence", "action", "condition"]
            # Remove non-sequence composite nodes from schema
            if "composite_nodes" in data:
                composite = data["composite_nodes"]
                data["composite_nodes"] = {"sequence": composite.get("sequence", {})}

        return data


class PyRoboSimWorldProvider(WorldEntitiesProvider):
    """Provides PyRoboSim world entities (vocabulary) for validation."""

    def __init__(self, world_file: str | None = None):
        """
        Initialize world provider.

        Args:
            world_file: Default world file to load for vocabulary
        """
        self.default_world_file = world_file

    def get_entities(
        self,
        mode: str = "vocab",
        world_file: str | None = None,
        control_url: str | None = None,
        robot: str | None = None,
    ) -> dict[str, Any]:
        """
        Return world entities for vocabulary validation.

        For now, returns vocab mode only (names/categories, no ground-truth).
        Future: Can load from world_file or query control_url for full/observed modes.

        Args:
            mode: "vocab" (names only), "observed" (robot knowledge), "full" (ground truth)
            world_file: Path to world YAML file (optional)
            control_url: URL to control server (optional)
            robot: Robot name (optional)

        Returns:
            Dictionary with: rooms, locations, objects, object_categories, etc.
        """
        # For vocab mode, return world vocabulary
        # This could be loaded from a world file or queried from a running simulation

        # Use provided world_file or fall back to default
        world_file = world_file or self.default_world_file

        if world_file:
            # Load world and extract vocabulary
            from pyrobosim.core import WorldYamlLoader
            from pyrobosim.mcp.world_entities import build_world_entities as _build_entities

            world = WorldYamlLoader().from_file(world_file)
            entities = _build_entities(
                world,
                mode=mode,
                robot=world.robots[0] if robot and world.robots else None,
                allow_full=(mode == "full"),
            )
            world.shutdown()
            return entities

        # Default: return vocab-only mode with no specific world loaded
        return {
            "mode": "vocab",
            "rooms": [],
            "locations": [],
            "object_spawns": [],
            "object_categories": [],
            "objects": [],
            "hallways": [],
        }
