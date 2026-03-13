"""PyRoboSim-specific MCP providers."""

from __future__ import annotations

import json
import urllib.request
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

    def __init__(self, world_file: str | None = None, control_url: str | None = None):
        """
        Initialize world provider.

        Args:
            world_file: Default world file to load for vocabulary
            control_url: Default URL of sim server for live vocabulary
        """
        self.default_world_file = world_file
        self.default_control_url = control_url

    def get_entities(
        self,
        mode: str = "vocab",
        world_file: str | None = None,
        control_url: str | None = None,
        robot: str | None = None,
    ) -> dict[str, Any]:
        """
        Return world entities for vocabulary validation.

        Priority order:
        1. If control_url provided, query running sim server for live vocabulary
        2. If world_file provided, load from file
        3. Fall back to empty vocab

        Args:
            mode: "vocab" (names only), "observed" (robot knowledge), "full" (ground truth)
            world_file: Path to world YAML file (optional)
            control_url: URL to control server (optional, e.g., http://localhost:9001)
            robot: Robot name (optional)

        Returns:
            Dictionary with: rooms, locations, objects, object_categories, etc.
        """
        # Priority 1: Query control server if URL provided
        control_url = control_url or self.default_control_url
        if control_url:
            try:
                request = urllib.request.Request(
                    control_url.rstrip("/") + "/world_entities",
                    data=json.dumps({
                        "mode": mode,
                        "robot": robot,
                        "allow_full": (mode == "full"),
                    }).encode("utf-8"),
                    headers={"Content-Type": "application/json", "Accept": "application/json"},
                    method="POST",
                )
                with urllib.request.urlopen(request, timeout=5) as response:
                    data = response.read().decode("utf-8")
                    return json.loads(data)
            except Exception as e:
                # If control server query fails, fall through to file-based approach
                print(f"Warning: Failed to query control server at {control_url}: {e}")
                print("Falling back to world file...")

        # Priority 2: Load from world file
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

        # Priority 3: Default empty vocab
        return {
            "mode": "vocab",
            "rooms": [],
            "locations": [],
            "object_spawns": [],
            "object_categories": [],
            "objects": [],
            "hallways": [],
        }
