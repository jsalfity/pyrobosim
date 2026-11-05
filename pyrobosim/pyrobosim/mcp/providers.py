"""PyRoboSim-specific MCP providers."""

from __future__ import annotations

import json
import urllib.request
from pathlib import Path
from typing import Any

import yaml

from bt_mcp.providers import BTSchemaProvider, RootstocksProvider, SkillProvider, WorldEntitiesProvider

from .validation import validate_bt_xml, vocabulary_from_entities

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
                "params": skill.get("params", {}),
            }
            if "outputs" in skill:
                formatted["outputs"] = skill["outputs"]
            formatted_skills.append(formatted)

        return formatted_skills


class PyRoboSimBTSchemaProvider(BTSchemaProvider):
    """Provides the PyRoboSim BT format (py_trees XML) and validates trees against it."""

    def __init__(
        self,
        schema_file: Path | None = None,
        world_provider: WorldEntitiesProvider | None = None,
        check_vocabulary: bool = False,
    ):
        """
        Initialize BT schema provider.

        Args:
            schema_file: Path to bt_schema.yaml (defaults to data/bt_schema.yaml)
            world_provider: Source of the world vocabulary for the vocabulary check
            check_vocabulary: Check that skill inputs name things in the world
        """
        self.schema_file = schema_file or (_DATA_DIR / "bt_schema.yaml")
        self.world_provider = world_provider
        self.check_vocabulary = check_vocabulary

    def get_schema(self, restrict_control_flow: bool = False) -> dict[str, Any]:
        """
        Load and return BT schema from YAML.

        Args:
            restrict_control_flow: If True, offer Sequence as the only control node

        Returns:
            BT format specification
        """
        data = yaml.safe_load(self.schema_file.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            return {}

        if restrict_control_flow:
            data = data.copy()
            data["node_types"] = ["Sequence", "Check", "skill"]
            data["composites"] = {"Sequence": data.get("composites", {}).get("Sequence", {})}
            data.pop("decorators", None)

        return data

    def validate_bt(self, bt: str, restrict_control_flow: bool = False) -> list[dict[str, Any]]:
        """Issues with a tree, or [] when the robot would accept it (see mcp/validation.py)."""
        vocabulary = None
        if self.check_vocabulary and self.world_provider is not None:
            vocabulary = vocabulary_from_entities(self.world_provider.get_entities(mode="vocab"))
        return validate_bt_xml(bt, restrict_control_flow=restrict_control_flow, vocabulary=vocabulary)


class PyRoboSimRootstocksProvider(RootstocksProvider):
    """Provides PyRoboSim rootstock templates from YAML."""

    def __init__(self, rootstocks_file: Path | None = None):
        self.rootstocks_file = rootstocks_file or (_DATA_DIR / "rootstocks.yaml")

    def get_rootstocks(self) -> list[dict[str, Any]]:
        """Load and return rootstock templates from YAML."""
        data = yaml.safe_load(self.rootstocks_file.read_text(encoding="utf-8"))
        rootstocks = data.get("rootstocks", []) if isinstance(data, dict) else []
        return [rootstock for rootstock in rootstocks if isinstance(rootstock, dict) and rootstock.get("name")]


class PyRoboSimWorldProvider(WorldEntitiesProvider):
    """Provides PyRoboSim world entities (vocabulary) from sim_app server."""

    def __init__(self, control_url: str | None = None):
        """
        Initialize world provider.

        Args:
            control_url: URL of sim_app server for live vocabulary
        """
        self.default_control_url = control_url

    def get_entities(
        self,
        mode: str = "vocab",
        control_url: str | None = None,
        robot: str | None = None,
    ) -> dict[str, Any]:
        """
        Return world entities for vocabulary validation from sim_app server.

        Args:
            mode: "vocab" (names only), "observed" (robot knowledge), "full" (ground truth)
            control_url: URL to sim_app server (optional, uses default if not provided)
            robot: Robot name (optional)

        Returns:
            Dictionary with: rooms, locations, object_spawns, objects, object_categories, hallways

        Raises:
            Exception: If sim_app server query fails
        """
        control_url = control_url or self.default_control_url
        if not control_url:
            raise ValueError("sim_app URL required for world entities")

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
            raise Exception(f"Failed to query sim_app server at {control_url}: {e}") from e
