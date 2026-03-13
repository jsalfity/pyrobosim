#!/usr/bin/env python3
"""Simple example: Run a behavior tree with GUI visualization.

This example shows how to execute a BT JSON file with PyRoboSim
while displaying the robot's movements in the GUI and printing
the tree status to the console.

Usage:
    python run_bt_gui.py path/to/bt.json --world-file roscon_2024_workshop_world.yaml
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.behaviors import build_pyrobosim_tree_from_json


def main():
    parser = argparse.ArgumentParser(description="Run BT with GUI")
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
        default=1.0,
        help="Simulation speed (1.0 = realtime)",
    )
    parser.add_argument(
        "--tick-rate",
        type=float,
        default=0.5,
        help="Time between tree ticks (seconds)",
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
    print(f"Loaded BT: {bt_json.get('name', 'Unnamed')}\n")

    # Build tree
    tree = build_pyrobosim_tree_from_json(
        bt_json,
        robot=world.robots[0],
        realtime_factor=args.realtime_factor,
    )
    tree.setup_with_descendants()

    # Execute tree in background thread
    import py_trees

    def run_tree():
        """Tick tree until terminal state, printing status."""
        time.sleep(2)  # Wait for GUI to initialize
        print("Executing BT...\n")

        while tree.root.status not in (
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.FAILURE,
        ):
            tree.tick()
            print(py_trees.display.unicode_tree(tree.root, show_status=True))
            time.sleep(args.tick_rate)

        print(f"\nFinal Status: {tree.root.status}")

    threading.Thread(target=run_tree, daemon=True).start()

    # Start GUI (blocking)
    start_gui(world)


if __name__ == "__main__":
    main()
