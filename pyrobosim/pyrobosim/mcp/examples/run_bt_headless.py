#!/usr/bin/env python3
"""Simple example: Run a behavior tree headless (no GUI).

This is a minimal example showing how to execute a BT JSON file
with PyRoboSim without visualization.

Usage:
    python run_bt_headless.py path/to/bt.json --world-file roscon_2024_workshop_world.yaml
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from pyrobosim.core import WorldYamlLoader
from pyrobosim.behaviors import build_pyrobosim_tree_from_json


def main():
    parser = argparse.ArgumentParser(description="Run BT headless")
    parser.add_argument("bt_file", type=Path, help="Path to BT JSON file")
    parser.add_argument(
        "--world-file",
        type=str,
        default="roscon_2024_workshop_world.yaml",
        help="World YAML file (name or path)",
    )
    parser.add_argument(
        "--realtime-factor",
        type=float,
        default=0.0,
        help="Simulation speed (0.0 = instant)",
    )
    args = parser.parse_args()

    # Load world
    world_file_path = Path(args.world_file).expanduser()
    if not world_file_path.is_absolute():
        if not world_file_path.exists():
            from pyrobosim.utils.general import get_data_folder

            pyrobosim_data_path = get_data_folder() / args.world_file
            if pyrobosim_data_path.exists():
                world_file_path = pyrobosim_data_path

    if not world_file_path.exists():
        print(f"ERROR: World file not found: {world_file_path}")
        return

    world = WorldYamlLoader().from_yaml(str(world_file_path))

    # Load BT JSON
    if not args.bt_file.exists():
        print(f"ERROR: BT file not found: {args.bt_file}")
        return

    bt_json = json.loads(args.bt_file.read_text())
    print(f"Loaded BT: {bt_json.get('name', 'Unnamed')}")

    # Build tree
    tree = build_pyrobosim_tree_from_json(
        bt_json,
        robot=world.robots[0],
        realtime_factor=args.realtime_factor,
    )
    tree.setup_with_descendants()

    print("\nExecuting BT...")

    # Execute tree (tick until terminal state)
    import py_trees

    while tree.root.status not in (
        py_trees.common.Status.SUCCESS,
        py_trees.common.Status.FAILURE,
    ):
        tree.tick()

    # Print result
    print(f"\nBT Status: {tree.root.status}")
    if tree.root.status == py_trees.common.Status.SUCCESS:
        print("SUCCESS")
        return 0
    else:
        print("FAILURE")
        return 1


if __name__ == "__main__":
    exit(main())
