"""PyRoboSim simulation app with GUI and HTTP control API.

This app provides a GUI visualization and HTTP control API for executing
behavior trees against a PyRoboSim world.

Usage:
    python -m pyrobosim.sim_app.server --world-file roscon_2024_workshop_world.yaml --port 9001

Endpoints:
    POST /run_bt - Execute a behavior tree (optional timeout_s: server-side deadline)
    POST /bt_status - Get status of a running BT
    POST /world_state - Get current world state (robot location, held object, objects)
    POST /world_entities - Get world vocabulary (locations, objects, categories)
    POST /reset_world - Reset world to initial state
    POST /reload_world - Reload world from file
    POST /cancel_bt - Stop one run ({"run_id": ...}) or every run ({"all": true});
                      answers once the run has stopped or the join bound expired
    GET|POST /live_runs - Number of BT runs whose ticking thread or action
                      workers are still alive

At most one behavior tree runs at a time. /run_bt, /reset_world and
/reload_world first stop and join any run that is still alive, and answer
HTTP 409 instead of proceeding if one will not stop.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
import traceback
import uuid
from contextlib import contextmanager
from enum import IntEnum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Iterator

import py_trees
from pyrobosim.behaviors import RobotActionBehavior, build_pyrobosim_tree_from_json
from pyrobosim.behaviors.pytree_json import cancel_navigation
from pyrobosim.core import World, WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.sim_app.world_entities import build_world_entities
from pyrobosim.utils.general import get_data_folder


DATA_FOLDER = get_data_folder()

# Stop limits, in the order they must nest:
#   HALT_LIMIT_S < CANCEL_JOIN_S < the harness client's 30 s urlopen timeout.
# HALT_LIMIT_S bounds stopping an abandoned tree's actions. It covers an RRT
# planning call (max_time 5 s in the workshop world), which cannot be
# interrupted, plus one executor step. CANCEL_JOIN_S bounds how long a
# /cancel_bt, /run_bt, /reset_world or /reload_world waits for a run to stop.
HALT_LIMIT_S = 10.0
CANCEL_JOIN_S = 20.0
# How long a control operation waits for another one to finish before it is
# refused with 409. The harness client is sequential, so this only matters for
# concurrent callers.
CONTROL_LOCK_WAIT_S = 5.0
# How long /live_runs waits for a run that has published its final status to
# finish returning, so a run that has just ended is not counted as live.
LIVE_RUNS_SETTLE_S = 1.0
# Granularity of every wait loop below. Waits block; nothing busy-spins.
POLL_S = 0.05

TERMINAL_STATUSES = ("SUCCESS", "FAILURE", "TIMEOUT", "CANCELED")


class RunStillAliveError(RuntimeError):
    """A BT run would not stop within its bound. Answered with HTTP 409."""


class ControlBusyError(RuntimeError):
    """Another start/reset/reload/cancel is in progress. Answered with HTTP 409."""


def _json_safe(value: Any) -> Any:
    """Convert runtime objects to JSON-serializable values."""
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, IntEnum):
        return value.name
    if isinstance(value, dict):
        return {str(key): _json_safe(inner) for key, inner in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(item) for item in value]
    status = getattr(value, "status", None)
    message = getattr(value, "message", None)
    if status is not None and hasattr(value, "is_success"):
        return {
            "status": _json_safe(status),
            "message": _json_safe(message),
            "success": bool(value.is_success()),
        }
    return repr(value)


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


def _action_nodes(tree: py_trees.trees.BehaviourTree) -> list[RobotActionBehavior]:
    """Every robot action node in `tree`."""
    return [node for node in tree.root.iterate() if isinstance(node, RobotActionBehavior)]


def _live_workers(actions: list[RobotActionBehavior]) -> list[threading.Thread]:
    return [worker for node in actions for worker in node.live_workers()]


def _halt_actions(
    actions: list[RobotActionBehavior],
    limit_s: float,
    root: py_trees.behaviour.Behaviour | None = None,
) -> bool:
    """Stop the actions of an abandoned tree and join their worker threads.

    If `root` is given, it is stopped with INVALID first, which calls
    RobotActionBehavior.terminate() on every running action; that must happen
    on the thread that ticks the tree (or after that thread has exited). Then,
    until every worker has exited: re-send the navigation cancel (a cancel
    that lands while a path is being planned is cleared when execution
    starts) and join the workers in short blocking waits.

    A worker that has exited cannot act any more, so the run counts as
    stopped even if the robot's busy flags were left set.

    :return: True if no worker thread started by `actions` is alive.
    """
    end = time.monotonic() + max(0.0, limit_s)
    for node in actions:
        node.request_halt(end)
    if root is not None:
        try:
            root.stop(py_trees.common.Status.INVALID)
        except Exception:
            traceback.print_exc()
    while True:
        alive_nodes = [node for node in actions if node.live_workers()]
        if not alive_nodes:
            return True
        for node in alive_nodes:
            cancel_navigation(node.robot)
        if time.monotonic() >= end:
            return False
        for worker in _live_workers(alive_nodes):
            worker.join(timeout=POLL_S)


class SimContext:
    """Context for managing world, robots, and BT executions."""

    def __init__(self, world: World, world_file: Path | None) -> None:
        self.world = world
        self.world_file = world_file
        self.bt_runs: dict[str, dict[str, Any]] = {}
        # Guards bt_runs. Held only briefly, and never while joining a thread:
        # the runner thread takes it to publish its tick count and status.
        self.lock = threading.Lock()
        # Serializes start_bt, reset_world, reload_world and cancel_bt. With
        # ThreadingHTTPServer each request has its own thread, so without it
        # the "nothing is running" check and the start or reset that follows
        # could interleave with another request. May be held while joining.
        self.control_lock = threading.Lock()

    @contextmanager
    def _exclusive(self, what: str) -> Iterator[None]:
        if not self.control_lock.acquire(timeout=CONTROL_LOCK_WAIT_S):
            raise ControlBusyError(
                f"cannot {what}: another start/reset/reload/cancel is in progress"
            )
        try:
            yield
        finally:
            self.control_lock.release()

    def _is_live(self, entry: dict[str, Any]) -> bool:
        """True while a run's ticking thread or any of its action workers is alive."""
        thread = entry.get("thread")
        if thread is not None and thread.is_alive():
            return True
        return bool(_live_workers(entry.get("actions") or []))

    def _live_run_ids(self) -> list[str]:
        with self.lock:
            entries = list(self.bt_runs.items())
        return [run_id for run_id, entry in entries if self._is_live(entry)]

    def _stop_run(self, run_id: str, end: float) -> dict[str, Any]:
        """Stop one run and wait for it, until the monotonic time `end`.

        The run's own thread performs the halt (it owns the tree) and then
        publishes CANCELED or TIMEOUT with halted=true. If that thread has
        already exited, any action worker it left behind is halted here.
        Caller must hold control_lock.
        """
        with self.lock:
            entry = self.bt_runs.get(run_id)
        if entry is None:
            return {"run_id": run_id, "status": "UNKNOWN", "alive": False, "halted": None}
        entry["cancel_event"].set()
        thread = entry.get("thread")
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.0, end - time.monotonic()))
        runner_alive = thread is not None and thread.is_alive()
        workers_stopped = True
        actions = entry.get("actions") or []
        if not runner_alive and _live_workers(actions):
            workers_stopped = _halt_actions(actions, end - time.monotonic())
        alive = runner_alive or not workers_stopped
        with self.lock:
            # halted describes the run as it is now: nothing it started is
            # still running.
            entry["halted"] = not alive
            if not alive:
                entry.pop("actions", None)
                if entry.get("status") not in TERMINAL_STATUSES:
                    # The ticking thread died without publishing a status.
                    entry["status"] = "CANCELED"
            return {
                "run_id": run_id,
                "status": entry.get("status"),
                "alive": alive,
                "halted": entry.get("halted"),
                "tick_count": entry.get("tick_count", 0),
            }

    def _stop_all_runs(self, join_s: float | None = None) -> dict[str, Any]:
        """Stop every live run, sharing one join bound. Caller must hold control_lock."""
        end = time.monotonic() + (CANCEL_JOIN_S if join_s is None else join_s)
        results = [self._stop_run(run_id, end) for run_id in self._live_run_ids()]
        return {
            "stopped": len(results),
            "still_alive": sum(1 for result in results if result["alive"]),
            "runs": results,
        }

    def _require_no_live_runs(self, what: str) -> None:
        """Stop-then-join before a start, reset or reload. Caller must hold control_lock."""
        outcome = self._stop_all_runs()
        if outcome["still_alive"]:
            raise RunStillAliveError(
                f"refusing to {what}: {outcome['still_alive']} earlier BT run(s) "
                f"did not stop within {CANCEL_JOIN_S:.0f} s; restart the simulator"
            )

    def cancel_bt(self, run_id: str | None = None) -> dict[str, Any]:
        """Stop one run, or every live run when `run_id` is None, and wait for it."""
        with self._exclusive("cancel"):
            if run_id is None:
                return self._stop_all_runs()
            return self._stop_run(run_id, time.monotonic() + CANCEL_JOIN_S)

    def live_runs(self) -> dict[str, Any]:
        """Report runs whose ticking thread or action workers are still alive."""
        with self.lock:
            entries = [(rid, entry) for rid, entry in self.bt_runs.items()]
        end = time.monotonic() + LIVE_RUNS_SETTLE_S
        for _, entry in entries:
            thread = entry.get("thread")
            if (
                entry.get("status") in TERMINAL_STATUSES
                and thread is not None
                and thread.is_alive()
            ):
                # Published its final status and is returning.
                thread.join(timeout=max(0.0, end - time.monotonic()))
        run_ids = [rid for rid, entry in entries if self._is_live(entry)]
        return {
            "live_runs": len(run_ids),
            "run_ids": run_ids,
            "robot_busy": any(robot.is_busy() for robot in self.world.robots),
        }

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
        policy_mode: bool = False,
        max_ticks: int = 0,
        timeout_s: float = 0.0,
    ) -> str:
        """Start a behavior tree execution in a background thread.

        Two termination regimes, because plan-shaped and policy-shaped BTs mean
        different things by a SUCCESS at the root:

        - Default (``policy_mode=False``): stop as soon as the root reports
          SUCCESS or FAILURE. Correct for a plan, whose root succeeds only when
          the last action of the task has completed.

        - ``policy_mode=True``: a root SUCCESS means "the highest-priority
          applicable rule ran this tick", which is progress, not completion, so
          keep ticking. The tree ends itself: a backward-chaining planner puts
          its goal condition as the first child of the root fallback, so once
          the goal holds that child succeeds without any action running, and the
          run stops. FAILURE still terminates immediately -- it means no rule is
          applicable, which is a genuine policy failure. ``max_ticks`` bounds
          livelock, where a policy oscillates between rules without progress
          (e.g. navigate/detect when the world does not match the action model).

        ``timeout_s`` > 0 sets a deadline, measured with time.monotonic() from
        the start of the run. A run still going at the deadline, like one
        that exhausts ``max_ticks`` or is cancelled, is abandoned: its running
        actions are cancelled, its action workers are joined, and only then is
        TIMEOUT (or CANCELED) published, with ``halted`` saying whether
        everything stopped. A tree that ends by itself is not affected.

        Any earlier run that is still alive is stopped and joined first.
        Raises RunStillAliveError if it does not stop.
        """
        with self._exclusive("start a BT"):
            self._require_no_live_runs("start a BT")
            return self._start_bt_locked(
                bt_json,
                robot_name,
                tick_ms,
                realtime_factor=realtime_factor,
                print_every=print_every,
                policy_mode=policy_mode,
                max_ticks=max_ticks,
                timeout_s=timeout_s,
            )

    def _start_bt_locked(
        self,
        bt_json: dict[str, Any],
        robot_name: str | None,
        tick_ms: int,
        realtime_factor: float,
        print_every: int,
        policy_mode: bool,
        max_ticks: int,
        timeout_s: float,
    ) -> str:
        robot = self.get_robot(robot_name)
        # The blackboard is process-global and is not otherwise reset, so
        # without this a run inherits keys written by whichever BT ran before
        # it. Detection results are the dangerous case: a stale
        # "<loc>_detections" entry can satisfy a condition in the next tree and
        # let it skip the detect that should have produced it. Clear before
        # building, so the tree's own `blackboard.initial` seeding still wins.
        # (Blackboard writes happen only in update(), on a run's own ticking
        # thread, and every earlier run's thread has been joined above, so no
        # earlier tree can write here after this clear.)
        py_trees.blackboard.Blackboard.clear()
        tree = build_pyrobosim_tree_from_json(bt_json, robot=robot, realtime_factor=realtime_factor)
        actions = _action_nodes(tree)
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

        def _goal_condition_satisfied() -> bool:
            """True if the root fallback succeeded via a pure condition.

            A backward-chaining planner places its goal check as the first child
            of the root fallback, so the run is complete exactly when the root
            reports SUCCESS and the child responsible is a condition rather than
            an action. Checking the responsible child (rather than scanning all
            action statuses) avoids being misled by an action that succeeded on
            an earlier tick and still carries a SUCCESS status.
            """
            for child in tree.root.children:
                if child.status != py_trees.common.Status.SUCCESS:
                    continue
                # The first successful child is the one the fallback returned
                # on. It completes the run only if no action ran underneath it.
                return not any(
                    isinstance(node, RobotActionBehavior) for node in child.iterate()
                )
            return False

        def _runner() -> None:
            tick_period_s = tick_ms / 1000.0
            deadline = time.monotonic() + timeout_s if timeout_s > 0 else None
            count = 0
            # None while the tree ends by itself; "TIMEOUT" or "CANCELED" when
            # it is abandoned.
            abandoned: str | None = None
            timeout_source: str | None = None
            tick_error: str | None = None

            while True:
                if cancel_event.is_set():
                    abandoned = "CANCELED"
                    break

                try:
                    tree.tick()
                except Exception:
                    tick_error = traceback.format_exc()
                    traceback.print_exc()
                    break
                count += 1
                _on_tick(count)

                status = tree.root.status
                if status == py_trees.common.Status.FAILURE:
                    break
                if status == py_trees.common.Status.SUCCESS:
                    # A plan's root succeeds only when the task is complete. A
                    # policy's root succeeds whenever any rule fires, so keep
                    # ticking unless the success came from the goal condition
                    # rather than from an action.
                    if not policy_mode or _goal_condition_satisfied():
                        break
                if max_ticks and count >= max_ticks:
                    abandoned = "TIMEOUT"
                    timeout_source = "tick_budget"
                    break
                if deadline is not None and time.monotonic() >= deadline:
                    abandoned = "TIMEOUT"
                    timeout_source = "server_deadline"
                    break

                time.sleep(tick_period_s)

            if tick_error is not None:
                # An exception out of tick() used to kill this thread and leave
                # the run RUNNING until the client gave up on it. Keep that
                # outcome (TIMEOUT at the deadline, or CANCELED), but keep this
                # thread so the run can still be stopped and joined.
                while not cancel_event.wait(POLL_S):
                    if deadline is not None and time.monotonic() >= deadline:
                        break
                abandoned = "CANCELED" if cancel_event.is_set() else "TIMEOUT"
                timeout_source = None if cancel_event.is_set() else "server_deadline"

            # Capture failure information
            failed_node = None
            if abandoned is None and tree.root.status == py_trees.common.Status.FAILURE:
                # Find the first failed node in the tree
                for node in tree.root.iterate():
                    if node.status == py_trees.common.Status.FAILURE:
                        failed_node = node.name
                        break

            # Nothing terminal is published until nothing the tree started is
            # still running, so a client reading world_state after the status
            # reads a world that has stopped changing.
            try:
                if abandoned is not None:
                    halted = _halt_actions(actions, HALT_LIMIT_S, root=tree.root)
                else:
                    # A tree that ended by itself has no action in flight, but
                    # its last worker may still be returning. Anything alive
                    # after a short join (an interrupted action) is stopped.
                    settle_end = time.monotonic() + 0.5
                    for worker in _live_workers(actions):
                        worker.join(timeout=max(0.0, settle_end - time.monotonic()))
                    halted = (not _live_workers(actions)) or _halt_actions(actions, HALT_LIMIT_S)
            except Exception:
                traceback.print_exc()
                halted = not _live_workers(actions)

            with self.lock:
                entry = self.bt_runs.get(run_id)
                if entry is None:
                    return
                entry["halted"] = halted
                if halted:
                    entry.pop("actions", None)
                if tick_error is not None:
                    entry["tick_error"] = tick_error
                if abandoned == "CANCELED":
                    entry["status"] = "CANCELED"
                elif abandoned == "TIMEOUT":
                    # Running out of time or ticks is not a tree status. Report
                    # it as such rather than letting the last tick's SUCCESS
                    # look like completion.
                    entry["status"] = "TIMEOUT"
                    entry["timeout_source"] = timeout_source
                    if timeout_source == "tick_budget":
                        entry["failed_node"] = "tick_budget_exhausted"
                else:
                    entry["status"] = tree.root.status.name
                    if failed_node:
                        entry["failed_node"] = failed_node

        thread = threading.Thread(target=_runner, daemon=True)
        with self.lock:
            # Registered with its thread before the thread starts, so a stop
            # can never see a run without the thread that must be joined.
            self.bt_runs[run_id] = {
                "status": "RUNNING",
                "cancel_event": cancel_event,
                "thread": thread,
                "actions": actions,
                "halted": None,
                "timeout_s": timeout_s,
                "tick_count": 0,
                "tree": "",
                "bb_keys": sorted(bb_keys),
                "failed_node": None,
            }
        thread.start()
        return run_id

    def bt_status(self, run_id: str) -> dict[str, Any]:
        """Get status of a running BT."""
        with self.lock:
            entry = self.bt_runs.get(run_id)
            if not entry:
                return {"status": "UNKNOWN"}
            blackboard = py_trees.blackboard.Blackboard()
            bb_values = _json_safe(dict(blackboard.storage))
            result = {
                "status": entry.get("status", "UNKNOWN"),
                "tick_count": entry.get("tick_count", 0),
                "tree": entry.get("tree", ""),
                "blackboard": bb_values,
            }
            # Add failed node if available
            if entry.get("failed_node"):
                result["failed_node"] = entry["failed_node"]
            # None while running. On a final status: True once nothing the
            # run started is still running, False if a stop ran out of time.
            result["halted"] = entry.get("halted")
            if entry.get("timeout_source"):
                result["timeout_source"] = entry["timeout_source"]
            if entry.get("tick_error"):
                result["tick_error"] = entry["tick_error"]
            return result

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
        # Container state. Without it the harness cannot score location_open /
        # location_closed and refuses to score the task at all, which aborts
        # any suite containing one (Hard15 hard07 and hard08). Location names
        # only: a spawn follows its parent, and the harness maps a spawn name
        # back to its container.
        open_locations = [
            loc.name for loc in self.world.locations if getattr(loc, "is_open", True)
        ]
        return {
            "robot": robot.name,
            "robot_location": location,  # Match expected field name from eval harness
            "location": location,
            "held_object": held,
            "objects": objects,
            "open_locations": open_locations,
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
        """Reset world to initial state, after stopping and joining any live BT run.

        A run left alive keeps the Robot it was built with, which reset
        replaces, and a reset that races a pick or place can hang in
        World.remove_location. Raises RunStillAliveError if a run will not stop.
        """
        with self._exclusive("reset the world"):
            self._require_no_live_runs("reset the world")
            return self.world.reset(deterministic=deterministic, seed=seed)

    def reload_world(self) -> bool:
        """Reload world from file, after stopping and joining any live BT run."""
        with self._exclusive("reload the world"):
            self._require_no_live_runs("reload the world")
            return self._reload_world_locked()

    def _reload_world_locked(self) -> bool:
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

    def _send_refused(self, exc: Exception) -> None:
        """409: a run is still alive, or another control operation holds the lock."""
        payload: dict[str, Any] = {"error": str(exc), "refused": type(exc).__name__}
        if self.context is not None:
            payload["live_runs"] = len(self.context._live_run_ids())
        self._send_json(payload, status=409)

    def do_GET(self) -> None:  # noqa: N802
        """Handle GET requests (read-only endpoints)."""
        if self.path != "/live_runs":
            self._send_json({"error": "not_found"}, status=404)
            return
        if self.context is None:
            self._send_json({"error": "no_context"}, status=500)
            return
        self._send_json(self.context.live_runs())

    def do_POST(self) -> None:  # noqa: N802
        """Handle POST requests."""
        if self.path not in (
            "/run_bt",
            "/bt_status",
            "/world_state",
            "/world_entities",
            "/reset_world",
            "/reload_world",
            "/cancel_bt",
            "/live_runs",
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

        if self.path == "/live_runs":
            self._send_json(self.context.live_runs())
            return

        if self.path == "/cancel_bt":
            run_id = data.get("run_id")
            if not run_id and not data.get("all"):
                self._send_json({"error": "run_id_or_all_required"}, status=400)
                return
            try:
                outcome = self.context.cancel_bt(None if data.get("all") else str(run_id))
            except ControlBusyError as exc:
                self._send_refused(exc)
                return
            self._send_json(outcome)
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
            except (RunStillAliveError, ControlBusyError) as exc:
                self._send_refused(exc)
                return
            except Exception as exc:
                self._send_json({"error": str(exc)}, status=400)
                return
            self._send_json({"reset": ok})
            return

        if self.path == "/reload_world":
            try:
                ok = self.context.reload_world()
            except (RunStillAliveError, ControlBusyError) as exc:
                self._send_refused(exc)
                return
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
        # Policy-shaped trees (e.g. from a backward-chaining planner) keep
        # ticking past a root SUCCESS; see SimContext.start_bt.
        policy_mode = bool(data.get("policy_mode", False))
        max_ticks = int(data.get("max_ticks", 0))
        # Server-side deadline in seconds (0 or absent: none, the old behaviour).
        try:
            timeout_s = float(data.get("timeout_s") or 0.0)
        except (TypeError, ValueError):
            self._send_json({"error": "timeout_s must be a number"}, status=400)
            return
        try:
            run_id = self.context.start_bt(
                bt_json,
                robot,
                tick_ms,
                realtime_factor=realtime_factor,
                print_every=print_every,
                policy_mode=policy_mode,
                max_ticks=max_ticks,
                timeout_s=timeout_s,
            )
        except (RunStillAliveError, ControlBusyError) as exc:
            self._send_refused(exc)
            return
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
