"""Helper function to get PyRoboSim rootstock templates."""

from __future__ import annotations

from typing import Any

from .providers import PyRoboSimRootstocksProvider


def get_rootstocks() -> list[dict[str, Any]]:
    """Load and return PyRoboSim rootstock templates."""
    provider = PyRoboSimRootstocksProvider()
    return provider.get_rootstocks()
