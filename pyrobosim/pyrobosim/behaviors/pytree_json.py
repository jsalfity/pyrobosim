"""PyRoboSim action wrapper for py_trees with JSON support."""

from __future__ import annotations

import threading
import time
from functools import partial
from typing import Any

import py_trees
from py_trees.json_bt import build_tree_from_json, extract_blackboard_keys, resolve_param

from pyrobosim.planning.actions import ExecutionResult, ExecutionStatus, TaskAction
from pyrobosim.utils.path import Path
from pyrobosim.utils.pose import Pose

#: Written to a `held_object` key when the gripper is empty. A blackboard key
#: that was never written also reads back as None, so an empty gripper and an
#: unwritten key would otherwise be indistinguishable to a condition node.
#: An explicit sentinel lets a BT test "not holding anything" as
#: {"key": ..., "operator": "==", "value": "__none__"}.
NONE_SENTINEL = "__none__"


class RobotActionBehavior(py_trees.behaviour.Behaviour):
    """Non-blocking wrapper around a PyRoboSim TaskAction."""

    def __init__(
        self,
        name: str,
        robot: Any,
        action_type: str,
        params: dict[str, Any] | None = None,
        outputs: dict[str, str] | None = None,
        realtime_factor: float = 1.0,
    ) -> None:
        super().__init__(name)
        self.robot = robot
        self.action_type = action_type
        self.params = params or {}
        self.outputs = outputs or {}
        self.realtime_factor = realtime_factor

        self.blackboard = py_trees.blackboard.Blackboard()
        self._bb_client = py_trees.blackboard.Client(name=f"{name}_client")
        for key in extract_blackboard_keys(self.params):
            self._bb_client.register_key(key=key, access=py_trees.common.Access.READ)
        for key in self.outputs.values():
            self._bb_client.register_key(key=key, access=py_trees.common.Access.WRITE)

        self._thread: threading.Thread | None = None
        # Every worker this node started that may still be running. A node
        # re-entered by a retry or a fallback starts a new worker, and an
        # earlier one that was interrupted can still be alive.
        self._workers: list[threading.Thread] = []
        self._result: ExecutionResult | None = None
        # Bumped by each initialise(), so a worker from an earlier entry cannot
        # hand its result to the current one.
        self._generation = 0
        # Set only by request_halt(). While None, terminate() does nothing and
        # an interrupted action runs to completion, as it always has.
        self._halt_deadline: float | None = None
        self._lock = threading.Lock()

    def initialise(self) -> None:
        # Custom resolvers for PyRoboSim types
        custom_resolvers = {
            "pose": _pose_from_dict,
            "path": _path_from_list,
        }

        resolved_params = {
            key: resolve_param(value, self.blackboard, custom_resolvers)
            for key, value in self.params.items()
        }
        action = _build_task_action(self.action_type, resolved_params)
        robot = self.robot

        with self._lock:
            self._generation += 1
            generation = self._generation
            self._result = None

        def _run_action() -> None:
            result: ExecutionResult | None = None
            try:
                result = robot.execute_action(action, realtime_factor=self.realtime_factor)
            except Exception as exc:
                # Robot.execute_action has no try/finally. Without this, an
                # exception (pick with no object, place with a dict pose) left
                # executing_action set forever: the node reported RUNNING
                # forever and robot.cancel_actions() could never return.
                result = ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE,
                    message=f"{action.type} raised {exc!r}",
                )
            finally:
                _release_robot_flags(robot, action)
                if result is None:
                    result = ExecutionResult(
                        status=ExecutionStatus.EXECUTION_FAILURE,
                        message=f"{action.type} worker exited without a result",
                    )
                with self._lock:
                    if generation == self._generation:
                        self._result = result

        thread = threading.Thread(target=_run_action, daemon=True)
        self._workers = [worker for worker in self._workers if worker.is_alive()]
        self._workers.append(thread)
        self._thread = thread
        thread.start()

    def update(self) -> py_trees.common.Status:
        # Read the worker's result before looking at the robot's busy flags.
        # A finished worker has always cleared the flags it set, so for a
        # normal action this is the same as checking is_busy() first; it
        # differs only when a flag is held by something else, which must not
        # keep this node RUNNING forever.
        with self._lock:
            result = self._result

        if result is None:
            return py_trees.common.Status.RUNNING

        self._write_outputs(result)
        return (
            py_trees.common.Status.SUCCESS
            if result.is_success()
            else py_trees.common.Status.FAILURE
        )

    def live_workers(self) -> list[threading.Thread]:
        """Worker threads started by this node that are still running."""
        return [worker for worker in list(self._workers) if worker.is_alive()]

    def request_halt(self, deadline: float) -> None:
        """Arm terminate() to stop this node's action, finishing by `deadline`.

        `deadline` is a time.monotonic() value. Called by the executor when it
        abandons a tree (timeout or cancel) just before stopping the root.
        """
        self._halt_deadline = deadline

    def terminate(self, new_status: py_trees.common.Status) -> None:
        # py_trees calls terminate() on every completion and on every priority
        # interrupt. Only a halt requested by the executor stops anything, so
        # a tree that finishes by itself runs exactly as before.
        deadline = self._halt_deadline
        if new_status != py_trees.common.Status.INVALID or deadline is None:
            return
        self.halt(max(0.0, deadline - time.monotonic()))

    def halt(self, limit_s: float) -> bool:
        """Cancel this node's in-flight action and wait for its workers.

        Navigation is the only interruptible action. The cancel flag is
        re-sent on every pass, because PathExecutor.execute() clears one that
        arrives while the path is still being planned. Pick, place, detect,
        open and close are short and run to completion. Returns True once no
        worker started by this node is alive; after that nothing it started
        can move the robot or write the blackboard.
        """
        end = time.monotonic() + max(0.0, limit_s)
        while True:
            alive = self.live_workers()
            if not alive:
                return True
            cancel_navigation(self.robot)
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                return False
            for worker in alive:
                worker.join(timeout=min(0.05, remaining))

    def _write_outputs(self, result: ExecutionResult) -> None:
        for value_name, bb_key in self.outputs.items():
            if value_name == "status":
                self._bb_client.set(bb_key, result.status.name)
            elif value_name == "message":
                self._bb_client.set(bb_key, result.message)
            elif value_name == "result":
                self._bb_client.set(bb_key, result)
            elif value_name == "detected_objects":
                detected = [obj.name for obj in self.robot.last_detected_objects]
                self._bb_client.set(bb_key, detected)
            elif value_name == "detected_categories":
                # Instance names ("bread0") are what detected_objects reports,
                # but task prompts and the world vocabulary speak in categories
                # ("bread"). Publishing categories separately lets a BT test
                # "did I see any bread?" without having to guess the instance
                # suffix. detected_objects is left unchanged so existing trees
                # keep working.
                self._bb_client.set(
                    bb_key,
                    sorted({obj.category for obj in self.robot.last_detected_objects}),
                )
            elif value_name == "battery_level":
                self._bb_client.set(bb_key, self.robot.battery_level)
            elif value_name == "last_nav_result":
                self._bb_client.set(bb_key, self.robot.last_nav_result)
            elif value_name == "robot_location":
                # Report the name an operator (and the world vocabulary) would
                # use. robot.location resolves to the object spawn the robot is
                # standing at -- "pantry_storage" -- whereas the vocabulary and
                # every task prompt say "pantry". Publish the parent's name so
                # a condition can be written against the vocabulary the
                # contract exposes.
                location = self.robot.location
                parent = getattr(location, "parent", None)
                name = getattr(parent, "name", None) or getattr(location, "name", None)
                if name is None and location is not None:
                    name = str(location)
                self._bb_client.set(bb_key, name)
            elif value_name == "held_object":
                held = self.robot.manipulated_object
                self._bb_client.set(bb_key, getattr(held, "name", None) or NONE_SENTINEL)
            elif value_name == "open_locations":
                # Names of locations currently open. Container state is not
                # otherwise observable, and a plan that must open a fridge
                # before reaching into it needs to test whether it already did.
                names = sorted(
                    loc.name
                    for loc in getattr(self.robot.world, "locations", [])
                    if getattr(loc, "is_open", False)
                )
                self._bb_client.set(bb_key, names)
            elif value_name == "objects_here":
                # Categories of objects currently at the robot's location.
                # Lets a BT check "is there a soda here?" after placing one,
                # which is the observable counterpart of an On(obj,loc) fact.
                location = self.robot.location
                parent = getattr(location, "parent", None)
                here = getattr(parent, "name", None) or getattr(location, "name", None)
                cats = sorted(
                    {
                        obj.category
                        for obj in getattr(self.robot.world, "objects", [])
                        if _object_location_name(obj) == here
                    }
                )
                self._bb_client.set(bb_key, cats)
            elif value_name == "held_category":
                # Category counterpart to held_object, for the same reason
                # detected_categories exists: the world holds "bread0" while
                # prompts and the vocabulary say "bread".
                held = self.robot.manipulated_object
                self._bb_client.set(bb_key, getattr(held, "category", None) or NONE_SENTINEL)


def cancel_navigation(robot: Any) -> None:
    """Ask the robot's path executor to stop, if a navigation is in progress."""
    executor = getattr(robot, "path_executor", None)
    if getattr(robot, "executing_nav", False) and executor is not None:
        executor.cancel_execution = True


def _release_robot_flags(robot: Any, action: TaskAction) -> None:
    """Clear the busy flags that `robot.execute_action(action)` set, if it raised.

    execute_action clears current_action and executing_action only on a normal
    return, so after one does they are already clear and nothing happens here.
    `current_action is action` means this call set them and nobody replaced
    them since. For a navigate that raised on this thread nothing is moving
    (the NavRunner is started only by the signal that follows the checks that
    can raise), so executing_nav is cleared too.
    """
    if getattr(robot, "current_action", None) is not action:
        return
    if action.type == "navigate":
        robot.executing_nav = False
    robot.current_action = None
    robot.executing_action = False


def _object_location_name(obj: Any) -> str | None:
    """Name of the location holding `obj`, at vocabulary granularity.

    Objects sit in an ObjectSpawn ("pantry_storage"), whose parent is the
    location the vocabulary names ("pantry").
    """
    spawn = getattr(obj, "parent", None)
    parent = getattr(spawn, "parent", None)
    return getattr(parent, "name", None) or getattr(spawn, "name", None)


def _pose_from_dict(data: dict[str, Any]) -> Pose:
    """Convert dict to PyRoboSim Pose."""
    return Pose(
        x=data.get("x", 0.0),
        y=data.get("y", 0.0),
        yaw=data.get("yaw", 0.0),
    )


def _path_from_list(points: list[dict[str, Any]]) -> Path:
    """Convert list of dicts to PyRoboSim Path."""
    poses = [_pose_from_dict(p) for p in points]
    return Path(poses=poses)


def _unwrap_param_value(value: Any) -> Any:
    """Unwrap MCP-style literal payloads into plain Python values."""
    if isinstance(value, dict):
        if set(value.keys()) == {"literal"}:
            return _unwrap_param_value(value["literal"])
        return {key: _unwrap_param_value(inner) for key, inner in value.items()}
    if isinstance(value, list):
        return [_unwrap_param_value(item) for item in value]
    return value


def _build_task_action(action_type: str, params: dict[str, Any]) -> TaskAction:
    """Build PyRoboSim TaskAction from parameters."""
    normalized = {key: _unwrap_param_value(value) for key, value in params.items()}
    return TaskAction(
        type=action_type,
        robot=normalized.get("robot"),
        object=normalized.get("object"),
        room=normalized.get("room"),
        source_location=normalized.get("source_location"),
        target_location=normalized.get("target_location"),
        pose=normalized.get("pose"),
        path=normalized.get("path", Path()),
        cost=normalized.get("cost"),
    )


def pyrobosim_action_factory(node_spec: dict[str, Any], robot: Any, realtime_factor: float = 1.0) -> RobotActionBehavior:
    """
    Factory function for creating PyRoboSim action nodes.

    Use with py_trees.json_bt.build_tree_from_json():
        from py_trees.json_bt import build_tree_from_json
        from pyrobosim.behaviors.local_bt import pyrobosim_action_factory
        from functools import partial

        factory = partial(pyrobosim_action_factory, robot=robot, realtime_factor=1.0)
        tree = build_tree_from_json(bt_json, action_factory=factory)
    """
    action_type = node_spec.get("action")
    if not action_type:
        raise ValueError("Action node missing action type.")

    return RobotActionBehavior(
        name=node_spec.get("name", action_type),
        robot=robot,
        action_type=action_type,
        params=node_spec.get("params", {}),
        outputs=node_spec.get("outputs", {}),
        realtime_factor=realtime_factor,
    )


# Convenience functions that use the new py_trees.json_bt module
def build_pyrobosim_tree_from_json(
    bt_json: dict[str, Any],
    robot: Any,
    realtime_factor: float = 1.0,
) -> py_trees.trees.BehaviourTree:
    """
    Build a PyRoboSim behavior tree from JSON specification.

    Convenience wrapper around py_trees.json_bt.build_tree_from_json with
    PyRoboSim action factory.

    Args:
        bt_json: JSON behavior tree specification
        robot: PyRoboSim robot instance
        realtime_factor: Simulation speed factor

    Returns:
        Executable PyTrees BehaviourTree
    """
    factory = partial(pyrobosim_action_factory, robot=robot, realtime_factor=realtime_factor)
    return build_tree_from_json(bt_json, action_factory=factory)
