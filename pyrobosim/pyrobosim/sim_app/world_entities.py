"""World entities extraction for sim app server."""

from __future__ import annotations

from typing import Any

from pyrobosim.core import World


def build_world_entities(
    world: World,
    mode: str = "vocab",
    robot: Any = None,
    allow_full: bool = False,
) -> dict[str, Any]:
    """
    Build world entities dictionary from a PyRoboSim world.

    Args:
        world: PyRoboSim World instance
        mode: "vocab" (names only), "observed" (robot knowledge), or "full" (ground truth)
        robot: Robot instance (required for "observed" mode)
        allow_full: Whether to allow "full" mode (default: False)

    Returns:
        Dictionary containing world entities (rooms, locations, objects, etc.)
    """
    if mode == "full" and not allow_full:
        raise ValueError("Full mode not allowed. Set allow_full=True to enable.")

    if mode == "observed" and robot is None:
        raise ValueError("Robot required for observed mode")

    entities: dict[str, Any] = {"mode": mode}

    # Rooms
    entities["rooms"] = [room.name for room in world.rooms]

    # Locations (navigate to these to interact with objects)
    entities["locations"] = [loc.name for loc in world.locations]
    entities["object_spawns"] = [spawn.name for spawn in world.object_spawns]

    # Object categories - extract from existing objects
    categories = set()
    for obj in world.objects:
        if obj.category:
            categories.add(obj.category)
    entities["object_categories"] = sorted(categories)

    # Objects
    if mode == "observed":
        # Only objects the robot knows about
        if robot and hasattr(robot, "known_objects"):
            entities["objects"] = [obj.name for obj in robot.known_objects]
        else:
            entities["objects"] = []
    else:
        # All objects in the world
        entities["objects"] = [obj.name for obj in world.objects]

    # Hallways
    entities["hallways"] = [hall.name for hall in world.hallways]

    return entities
