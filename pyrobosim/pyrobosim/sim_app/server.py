"""PyRoboSim simulation app with GUI and HTTP control API.

This app provides a GUI visualization and HTTP control API for executing
behavior trees against a PyRoboSim world.

Usage:
    python -m pyrobosim.sim_app.server --world-file roscon_2024_workshop_world.yaml --port 9001

Endpoints:
    POST /run_bt - Execute a behavior tree
    POST /bt_status - Get status of a running BT
    POST /world_state - Get current world state (robot location, held object, objects)
    POST /world_entities - Get world vocabulary (locations, objects, categories)
    POST /reset_world - Reset world to initial state
    POST /reload_world - Reload world from file
"""

from __future__ import annotations

import argparse
import json
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import py_trees
from pyrobosim.behaviors import build_pyrobosim_tree_from_json
from pyrobosim.core import World, WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.sim_app.world_entities import build_world_entities
from pyrobosim.utils.general import get_data_folder


DATA_FOLDER = get_data_folder()


def _collect_blackboard_keys(node: dict[str, Any], keys: set[str]) -> None:
    """Recursively collect blackboard keys from BT node."""
    node_type = node.get("type")
    if node_type == "action":
        outputs = node.get("outputs", {})
        if isinstance(outputs, dict):
            for key in outputs.values():
                if isinstance(key, str):
                    keys.add(key)
    if node_type == "condition":
        key = node.get("key")
        if isinstance(key, str):
            keys.add(key)
    if node_type == "decorator":
        child = node.get("child")
        if isinstance(child, dict):
            _collect_blackboard_keys(child, keys)
        return
    for child in node.get("children", []):
        if isinstance(child, dict):
            _collect_blackboard_keys(child, keys)


class SimContext:
    """Context for managing world, robots, and BT executions."""

    def __init__(self, world: World, world_file: Path | None) -> None:
        self.world = world
        self.world_file = world_file
        self.bt_runs: dict[str, dict[str, Any]] = {}
        self.lock = threading.Lock()

    def get_robot(self, name: str | None = None):
        """Get robot by name or return default robot."""
        if name:
            robot = self.world.get_robot_by_name(name)
            if robot is None:
                names = [r.name for r in self.world.robots]
                raise ValueError(f"Robot '{name}' not found. Available: {names}")
            return robot
        if not self.world.robots:
            raise ValueError("No robots in world")
        return self.world.robots[0]

    def start_bt(
        self,
        bt_json: dict[str, Any],
        robot_name: str | None,
        tick_ms: int,
        realtime_factor: float = 1.0,
        print_every: int = 1,
    ) -> str:
        """Start a behavior tree execution in a background thread."""
        robot = self.get_robot(robot_name)
        tree = build_pyrobosim_tree_from_json(bt_json, robot=robot, realtime_factor=realtime_factor)
        cancel_event = threading.Event()
        run_id = str(uuid.uuid4())
        bb_keys: set[str] = set()
        root_spec = bt_json.get("root")
        if isinstance(root_spec, dict):
            _collect_blackboard_keys(root_spec, bb_keys)

        def _on_tick(count: int) -> None:
            tree_text = ""
            if print_every > 0 and count % print_every == 0:
                tree_text = py_trees.display.unicode_tree(tree.root, show_status=True)
            with self.lock:
                if run_id in self.bt_runs:
                    self.bt_runs[run_id]["tick_count"] = count
                    if tree_text:
                        self.bt_runs[run_id]["tree"] = tree_text

        def _runner() -> None:
            import time
            tick_period_s = tick_ms / 1000.0
            count = 0

            while tree.root.status not in (
                py_trees.common.Status.SUCCESS,
                py_trees.common.Status.FAILURE,
            ):
                if cancel_event.is_set():
                    with self.lock:
                        if run_id in self.bt_runs:
                            self.bt_runs[run_id]["status"] = "CANCELED"
                    return

                tree.tick()
                count += 1
                _on_tick(count)
                time.sleep(tick_period_s)

            with self.lock:
                if run_id in self.bt_runs:
                    self.bt_runs[run_id]["status"] = tree.root.status.name

        with self.lock:
            self.bt_runs[run_id] = {
                "status": "RUNNING",
                "cancel_event": cancel_event,
                "tick_count": 0,
                "tree": "",
                "bb_keys": sorted(bb_keys),
            }
        thread = threading.Thread(target=_runner, daemon=True)
        thread.start()
        with self.lock:
            self.bt_runs[run_id]["thread"] = thread
        return run_id

    def bt_status(self, run_id: str) -> dict[str, Any]:
        """Get status of a running BT."""
        with self.lock:
            entry = self.bt_runs.get(run_id)
            if not entry:
                return {"status": "UNKNOWN"}
            blackboard = py_trees.blackboard.Blackboard()
            bb_values = dict(blackboard.storage)
            return {
                "status": entry.get("status", "UNKNOWN"),
                "tick_count": entry.get("tick_count", 0),
                "tree": entry.get("tree", ""),
                "blackboard": bb_values,
            }

    def world_state(self, robot_name: str | None = None) -> dict[str, Any]:
        """Get current world state."""
        robot = self.get_robot(robot_name)
        held = None
        if robot.manipulated_object is not None:
            held = {
                "name": robot.manipulated_object.name,
                "category": robot.manipulated_object.category,
            }
        objects = []
        for obj in self.world.objects:
            parent = obj.parent.name if obj.parent is not None else None
            objects.append(
                {
                    "name": obj.name,
                    "category": obj.category,
                    "location": parent,
                }
            )
        location = robot.location.name if robot.location is not None else None
        return {
            "robot": robot.name,
            "robot_location": location,  # Match expected field name from eval harness
            "location": location,
            "held_object": held,
            "objects": objects,
        }

    def world_entities(
        self,
        mode: str = "vocab",
        robot_name: str | None = None,
        allow_full: bool = False,
    ) -> dict[str, Any]:
        """Get world entities/vocabulary for BT generation."""
        robot = None
        if mode == "observed" or robot_name:
            robot = self.get_robot(robot_name)
        return build_world_entities(self.world, mode=mode, robot=robot, allow_full=allow_full)

    def reset_world(self, deterministic: bool = False, seed: int = -1) -> bool:
        """Reset world to initial state."""
        return self.world.reset(deterministic=deterministic, seed=seed)

    def reload_world(self) -> bool:
        """Reload world from file."""
        if self.world_file is None:
            raise ValueError("No world_file configured for reload.")
        gui = self.world.gui
        new_world = WorldYamlLoader().from_file(self.world_file)
        self.world = new_world
        if gui is not None:
            gui.set_world(new_world)
            # Reinitialize robot visualization including sensor artists
            gui.canvas.show_robots()
        return True


class ControlHandler(BaseHTTPRequestHandler):
    """HTTP request handler for control endpoints."""

    context: SimContext | None = None

    def log_message(self, format: str, *args: Any) -> None:
        """Override to reduce logging noise."""
        pass

    def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        """Send JSON response."""
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:  # noqa: N802
        """Handle POST requests."""
        if self.path not in (
            "/run_bt",
            "/bt_status",
            "/world_state",
            "/world_entities",
            "/reset_world",
            "/reload_world",
        ):
            self._send_json({"error": "not_found"}, status=404)
            return

        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length).decode("utf-8") if length > 0 else "{}"
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            self._send_json({"error": "invalid_json"}, status=400)
            return

        if self.context is None:
            self._send_json({"error": "no_context"}, status=500)
            return

        if self.path == "/bt_status":
            run_id = data.get("run_id")
            if not run_id:
                self._send_json({"error": "run_id_required"}, status=400)
                return
            self._send_json(self.context.bt_status(run_id))
            return

        if self.path == "/world_state":
            robot = data.get("robot")
            try:
                state = self.context.world_state(robot)
            except Exception as exc:
                self._send_json({"error": str(exc)}, status=400)
                return
            self._send_json(state)
            return

        if self.path == "/world_entities":
            mode = data.get("mode", "vocab")
            robot = data.get("robot")
            allow_full = bool(data.get("allow_full", False))
            try:
                payload = self.context.world_entities(
                    mode=mode, robot_name=robot, allow_full=allow_full
                )
            except Exception as exc:
                self._send_json({"error": str(exc)}, status=400)
                return
            self._send_json(payload)
            return

        if self.path == "/reset_world":
            deterministic = bool(data.get("deterministic", False))
            seed = int(data.get("seed", -1))
            try:
                ok = self.context.reset_world(deterministic=deterministic, seed=seed)
            except Exception as exc:
                self._send_json({"error": str(exc)}, status=400)
                return
            self._send_json({"reset": ok})
            return

        if self.path == "/reload_world":
            try:
                ok = self.context.reload_world()
            except Exception as exc:
                self._send_json({"error": str(exc)}, status=400)
                return
            self._send_json({"reload": ok})
            return

        # /run_bt endpoint
        bt_json = data.get("bt_json")
        if not isinstance(bt_json, dict):
            self._send_json({"error": "bt_json_required"}, status=400)
            return

        robot = data.get("robot")
        tick_ms = int(data.get("tick_ms", 100))
        realtime_factor = float(data.get("realtime_factor", 1.0))
        print_every = int(data.get("print_every", 1))
        try:
            run_id = self.context.start_bt(
                bt_json, robot, tick_ms, realtime_factor=realtime_factor, print_every=print_every
            )
        except Exception as exc:
            self._send_json({"error": str(exc)}, status=400)
            return
        self._send_json({"run_id": run_id})


def start_control_server(context: SimContext, host: str, port: int) -> ThreadingHTTPServer:
    """Start the control HTTP server in a background thread."""
    ControlHandler.context = context
    server = ThreadingHTTPServer((host, port), ControlHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"Control server listening on {host}:{port}")
    return server


def load_world(world_file: str | Path) -> tuple[World, Path]:
    """Load world from file."""
    loader = WorldYamlLoader()

    # Handle both absolute and relative paths
    if isinstance(world_file, str):
        world_file = Path(world_file)

    if world_file.is_absolute():
        world_path = world_file
    else:
        # Try data folder first
        world_path = DATA_FOLDER / world_file
        if not world_path.exists():
            # Try relative to current directory
            world_path = world_file

    if not world_path.exists():
        raise FileNotFoundError(f"World file not found: {world_path}")

    world = loader.from_file(world_path)
    return world, world_path


def main() -> None:
    """Main entry point for the sim app."""
    parser = argparse.ArgumentParser(
        description="PyRoboSim simulation app with GUI visualization and HTTP control API."
    )
    parser.add_argument(
        "--world-file",
        required=True,
        help="Path to YAML world file (absolute or relative to pyrobosim data folder)",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Control server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9001, help="Control server port (default: 9001)")
    args = parser.parse_args()

    print(f"Loading world from: {args.world_file}")
    world, world_file = load_world(args.world_file)

    print(f"World loaded with {len(world.robots)} robot(s)")
    context = SimContext(world, world_file)

    start_control_server(context, args.host, args.port)

    print("Starting GUI (this will block until GUI is closed)...")
    start_gui(world)


if __name__ == "__main__":
    main()
