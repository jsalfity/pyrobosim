#!/usr/bin/env python3
"""Test script for new py_trees.json_bt integration with PyRoboSim."""

import json
import sys
import threading
from pathlib import Path

# Add pyrobosim to path
sys.path.insert(0, str(Path(__file__).parent))

from pyrobosim.core import WorldYamlLoader
from pyrobosim.behaviors.local_bt import build_pyrobosim_tree_from_json
from pyrobosim.web import start_ui
import py_trees
import py_trees.display


def main():
    """Run a simple BT test."""
    # Load a simple world
    print("Loading world...")
    world_file = Path("/Users/jonathansalfity/Documents/dev/pyrobosim/pyrobosim/pyrobosim/data/roscon_2024_workshop_world.yaml")

    if not world_file.exists():
        print(f"World file not found: {world_file}")
        print("Looking for available world files...")
        data_dir = Path(__file__).parent / "pyrobosim" / "data"
        if data_dir.exists():
            yaml_files = list(data_dir.glob("*.yaml"))
            print(f"Found {len(yaml_files)} YAML files:")
            for f in yaml_files[:5]:
                print(f"  - {f.name}")
            if yaml_files:
                world_file = yaml_files[0]
                print(f"\nUsing: {world_file.name}")
        else:
            print(f"Data directory not found: {data_dir}")
            return

    world = WorldYamlLoader().from_file(world_file)

    # Get the first robot
    if not world.robots:
        print("No robots in world!")
        return

    robot = world.robots[0]
    print(f"Using robot: {robot.name}")
    print(f"Robot starting location: {robot.location}")

    # Print available locations
    print("\nAvailable locations:")
    for room in world.rooms:
        print(f"  Room: {room.name}")
    for loc in world.locations:
        print(f"  Location: {loc.name} (in {loc.parent.name})")

    # Load BT JSON
    bt_file = Path(__file__).parent / "test_bt_simple.json"
    print(f"\nLoading BT from: {bt_file}")
    with open(bt_file) as f:
        bt_json = json.load(f)

    print("BT structure:")
    print(json.dumps(bt_json, indent=2))

    # Build tree
    print("\nBuilding behavior tree...")
    try:
        tree = build_pyrobosim_tree_from_json(bt_json, robot=robot, realtime_factor=1.0)
        print("✓ Tree built successfully!")
    except Exception as e:
        print(f"✗ Failed to build tree: {e}")
        import traceback
        traceback.print_exc()
        return

    # Print tree structure
    print("\nTree structure:")
    print(tree.root)

    # Create a function to run the tree in a separate thread
    def run_tree():
        import time
        time.sleep(2)  # Wait for GUI to initialize
        print("\nTicking tree...\n")
        print("=" * 80)

        try:
            # Tick manually using native PyTrees to show progress
            tick_count = 0
            while tree.root.status not in (py_trees.common.Status.SUCCESS,
                                           py_trees.common.Status.FAILURE):
                tick_count += 1

                # Use native PyTrees tick method
                tree.tick()

                # Print tree status after each tick
                print(f"\n--- Tick {tick_count} ---")
                print(py_trees.display.unicode_tree(
                    tree.root,
                    show_status=True,
                    show_only_visited=False
                ))

                # Small delay between ticks
                time.sleep(0.5)

                # Safety limit
                if tick_count > 100:
                    print("Safety limit reached!")
                    break

            print("\n" + "=" * 80)
            print(f"\n✓ Tree execution completed with status: {tree.root.status}")
            print(f"Robot final location: {robot.location}")
        except Exception as e:
            print(f"✗ Tree execution failed: {e}")
            import traceback
            traceback.print_exc()

    # Start the behavior tree in a background thread
    bt_thread = threading.Thread(target=run_tree, daemon=True)
    bt_thread.start()

    # Start GUI visualization (this will block until GUI is closed)
    print("\nStarting web UI visualization...")
    print("The robot will start moving in 2 seconds...")
    print("Stop the server (Ctrl+C) to exit.")
    start_ui(world)


if __name__ == "__main__":
    main()
