"""Helper function to get PyRoboSim skill schemas for validation."""

from __future__ import annotations

from typing import Any

from .providers import PyRoboSimSkillProvider


def get_skill_schemas() -> list[dict[str, Any]]:
    """
    Load and return PyRoboSim skill schemas.

    Returns:
        List of skill schema dicts with name, description, parameters, outputs
    """
    provider = PyRoboSimSkillProvider()
    return provider.get_skills()
