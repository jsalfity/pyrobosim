"""PyRoboSim action wrapper for py_trees with JSON support."""

from __future__ import annotations

import threading
from functools import partial
from typing import Any

import py_trees
from py_trees.json_bt import build_tree_from_json, extract_blackboard_keys, resolve_param

from pyrobosim.planning.actions import ExecutionResult, TaskAction
from pyrobosim.utils.path import Path
from pyrobosim.utils.pose import Pose


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
        self._result: ExecutionResult | None = None
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

        def _run_action() -> None:
            result = self.robot.execute_action(action, realtime_factor=self.realtime_factor)
            with self._lock:
                self._result = result

        with self._lock:
            self._result = None
        self._thread = threading.Thread(target=_run_action, daemon=True)
        self._thread.start()

    def update(self) -> py_trees.common.Status:
        if self.robot.is_busy():
            return py_trees.common.Status.RUNNING

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
            elif value_name == "battery_level":
                self._bb_client.set(bb_key, self.robot.battery_level)
            elif value_name == "last_nav_result":
                self._bb_client.set(bb_key, self.robot.last_nav_result)


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
