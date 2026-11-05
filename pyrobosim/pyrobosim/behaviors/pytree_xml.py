"""PyRoboSim skills and blackboard checks as py_trees XML nodes.

Trees are written in py_trees' BehaviorTree XML format and built with its own
parser (py_trees.parsers.behaviour_tree_xml)::

    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <Sequence name="fetch" memory="true">
          <navigate name="go" target_location="pantry"/>
          <detect name="look" object="bread"/>
          <pick name="take" object="bread"/>
        </Sequence>
      </BehaviorTree>
    </root>

Composites (Sequence, Selector or Fallback, Parallel) and decorators (Retry,
Inverter, ...) are py_trees' own. This module supplies the leaves: one tag per
PyRoboSim skill, and a Check tag that compares a blackboard value.

Skill inputs are ports, so they take a literal (object="bread") or a blackboard
key (object="{target}"). Skill outputs are ports too: detected_categories="{seen}"
writes that output to /seen when the action finishes. The robot's state keys
(STATE_KEYS) are refreshed under their own names after every action, whether
or not the tree names them, so a Check can always read the current state.
"""

from __future__ import annotations

import functools
import json
import os
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from typing import Any

import py_trees
from py_trees.parsers.behaviour_tree_xml import parse_behaviour_tree_xml
from py_trees.ports import NoDataAvailable, PortInformation, PortsMixin

from pyrobosim.planning.actions import ExecutionResult, ExecutionStatus, TaskAction
from pyrobosim.utils.path import Path

#: Written to a `held_object` or `held_category` key when the gripper is empty.
#: A blackboard key that was never written also reads back as missing, so an
#: empty gripper and an unwritten key would otherwise be indistinguishable to a
#: Check. The sentinel lets a tree test "not holding anything" as
#: <Check key="held_category" operator="==" value="__none__"/>.
NONE_SENTINEL = "__none__"

#: Refreshed by every action when it finishes, so a Check can read the robot's
#: state without the tree wiring these outputs itself.
STATE_KEYS = ("robot_location", "held_category", "objects_here", "open_locations")

#: Every output a skill can write. Each is an output port.
OUTPUTS = (
    "status",
    "message",
    "result",
    "detected_objects",
    "detected_categories",
    "battery_level",
    "last_nav_result",
    "robot_location",
    "held_object",
    "held_category",
    "objects_here",
    "open_locations",
)

#: Operators a Check accepts.
OPERATORS = ("==", "!=", ">", ">=", "<", "<=", "in", "contains", "not_contains", "truthy")

_TEXT = PortInformation(data_type=str, required=False)
_ANY = PortInformation(data_type=Any, required=False)
_OUTPUT_PORTS = {name: _ANY for name in OUTPUTS}


class RobotActionBehavior(PortsMixin, py_trees.behaviour.Behaviour):
    """Non-blocking wrapper around a PyRoboSim TaskAction; one subclass per skill."""

    #: The TaskAction type this node runs, set by each skill subclass.
    ACTION: str = ""

    def __init__(
        self,
        name: str,
        robot: Any,
        realtime_factor: float = 1.0,
        behaviour_class_name: str | None = None,
    ) -> None:
        super().__init__(name=name, behaviour_class_name=behaviour_class_name)
        self.robot = robot
        self.action_type = self.ACTION
        self.realtime_factor = realtime_factor

        self._state = self.attach_blackboard_client(name=f"{name}_state")
        for key in STATE_KEYS:
            self._state.register_key(key=key, access=py_trees.common.Access.WRITE)
        # Output ports the tree wired to a key; filled in by setup_ports().
        self._wired_outputs: list[str] = []

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

    def setup_ports(
        self,
        port_remappings: dict | None = None,
        subtree_namespace: str = "/",
        logger: Any = None,
    ) -> None:
        super().setup_ports(port_remappings=port_remappings, subtree_namespace=subtree_namespace, logger=logger)
        self._wired_outputs = [port for port in (port_remappings or {}) if port in self.output_ports()]

    def params(self) -> dict[str, Any]:
        """The inputs this node was given, literal or read from the blackboard."""
        values: dict[str, Any] = {}
        for port in self.input_ports():
            try:
                values[port] = self.get_input(port)
            except NoDataAvailable:
                continue
        return values

    def output_keys(self) -> list[str]:
        """Blackboard keys this node writes: the state keys and its wired outputs."""
        return [f"/{key}" for key in STATE_KEYS] + [self._get_blackboard_key(port) for port in self._wired_outputs]

    def initialise(self) -> None:
        params = self.params()
        action = TaskAction(
            type=self.action_type,
            object=params.get("object"),
            target_location=params.get("target_location"),
            path=Path(),
        )
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
                # exception (pick with no object) left executing_action set
                # forever: the node reported RUNNING forever and
                # robot.cancel_actions() could never return.
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
        for key in STATE_KEYS:
            self._state.set(key, self._output(key, result))
        for port in self._wired_outputs:
            self._set_output(port, self._output(port, result))

    def _output(self, name: str, result: ExecutionResult) -> Any:
        """The value of one output after the action ran."""
        if name == "status":
            return result.status.name
        if name == "message":
            return result.message
        if name == "result":
            return result
        if name == "detected_objects":
            return [obj.name for obj in self.robot.last_detected_objects]
        if name == "detected_categories":
            # Instance names ("bread0") are what detected_objects reports, but
            # task prompts and the world vocabulary speak in categories ("bread").
            return sorted({obj.category for obj in self.robot.last_detected_objects})
        if name == "battery_level":
            return self.robot.battery_level
        if name == "last_nav_result":
            return self.robot.last_nav_result
        if name == "robot_location":
            # Report the name the world vocabulary uses. robot.location resolves
            # to the object spawn the robot stands at ("pantry_storage"); every
            # task prompt says "pantry".
            location = self.robot.location
            parent = getattr(location, "parent", None)
            location_name = getattr(parent, "name", None) or getattr(location, "name", None)
            if location_name is None and location is not None:
                location_name = str(location)
            return location_name
        if name == "held_object":
            return getattr(self.robot.manipulated_object, "name", None) or NONE_SENTINEL
        if name == "held_category":
            return getattr(self.robot.manipulated_object, "category", None) or NONE_SENTINEL
        if name == "open_locations":
            # Container state is not otherwise observable, and a tree that must
            # open a fridge before reaching into it needs to test whether it did.
            return sorted(
                loc.name
                for loc in getattr(getattr(self.robot, "world", None), "locations", [])
                if getattr(loc, "is_open", False)
            )
        if name == "objects_here":
            # Categories of objects at the robot's location: the observable
            # counterpart of an On(obj, loc) fact.
            location = self.robot.location
            parent = getattr(location, "parent", None)
            here = getattr(parent, "name", None) or getattr(location, "name", None)
            return sorted(
                {
                    obj.category
                    for obj in getattr(getattr(self.robot, "world", None), "objects", [])
                    if _object_location_name(obj) == here
                }
            )
        raise KeyError(f"unknown output {name!r}")


class Navigate(RobotActionBehavior, register=False):
    """Plan and follow a path to a location."""

    ACTION = "navigate"
    INPUT_PORTS = {"target_location": _TEXT}
    OUTPUT_PORTS = _OUTPUT_PORTS


class Pick(RobotActionBehavior, register=False):
    """Pick up an object at the current location."""

    ACTION = "pick"
    INPUT_PORTS = {"object": _TEXT}
    OUTPUT_PORTS = _OUTPUT_PORTS


class Place(RobotActionBehavior, register=False):
    """Place the held object at the current location."""

    ACTION = "place"
    INPUT_PORTS: dict = {}
    OUTPUT_PORTS = _OUTPUT_PORTS


class Detect(RobotActionBehavior, register=False):
    """Detect objects at the current location."""

    ACTION = "detect"
    INPUT_PORTS = {"object": _TEXT}
    OUTPUT_PORTS = _OUTPUT_PORTS


class Open(RobotActionBehavior, register=False):
    """Open the current or given location."""

    ACTION = "open"
    INPUT_PORTS = {"target_location": _TEXT}
    OUTPUT_PORTS = _OUTPUT_PORTS


class Close(RobotActionBehavior, register=False):
    """Close the current or given location."""

    ACTION = "close"
    INPUT_PORTS = {"target_location": _TEXT}
    OUTPUT_PORTS = _OUTPUT_PORTS


#: XML tag -> skill node class.
SKILLS: dict[str, type[RobotActionBehavior]] = {
    cls.ACTION: cls for cls in (Navigate, Pick, Place, Detect, Open, Close)
}


def _decode(value: str) -> Any:
    """A Check value is JSON when it parses as JSON (a list, a number), otherwise a string."""
    try:
        return json.loads(value)
    except ValueError:
        return value


class Check(PortsMixin, py_trees.behaviour.Behaviour, register=False):
    """SUCCESS if the blackboard value at `key` satisfies `operator` `value`, else FAILURE.

    A key that has not been written yet is not satisfied: it fails rather than
    raising, so a check on a value no action has produced yet simply fails.
    """

    INPUT_PORTS: dict = {}
    OUTPUT_PORTS: dict = {}

    def __init__(
        self,
        name: str,
        key: str,
        operator: str = "==",
        value: str = "",
        behaviour_class_name: str | None = None,
    ) -> None:
        if operator not in OPERATORS:
            raise ValueError(f"operator {operator!r} is not one of {', '.join(OPERATORS)}")
        super().__init__(name=name, behaviour_class_name=behaviour_class_name)
        self.key = key
        self.operator = operator
        self.value = _decode(value)
        self._reader = self.attach_blackboard_client(name=f"{name}_reader")
        self._reader.register_key(key=key, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            current = self._reader.get(self.key)
        except KeyError:
            return py_trees.common.Status.FAILURE
        if current is None:
            return py_trees.common.Status.FAILURE
        try:
            satisfied = _compare(self.operator, current, self.value)
        except TypeError:
            satisfied = False
        return py_trees.common.Status.SUCCESS if satisfied else py_trees.common.Status.FAILURE


def _compare(operator: str, current: Any, value: Any) -> bool:
    if operator == "==":
        return current == value
    if operator == "!=":
        return current != value
    if operator == ">":
        return current > value
    if operator == ">=":
        return current >= value
    if operator == "<":
        return current < value
    if operator == "<=":
        return current <= value
    if operator == "in":
        return current in value
    if operator == "contains":
        return value in current
    if operator == "not_contains":
        return value not in current
    if operator == "truthy":
        return bool(current)
    return False


def node_registry(robot: Any, realtime_factor: float = 1.0) -> dict[str, Any]:
    """XML tag -> constructor for every leaf a tree may use, bound to `robot`."""
    registry: dict[str, Any] = {
        tag: functools.partial(cls, robot=robot, realtime_factor=realtime_factor) for tag, cls in SKILLS.items()
    }
    registry["Check"] = Check
    return registry


def _main_tree_id(bt_xml: str) -> str | None:
    """The tree to run: `main_tree_to_execute`, or the only <BehaviorTree> when there is one."""
    root = ET.fromstring(bt_xml)
    if root.get("main_tree_to_execute"):
        return None
    trees = root.findall("BehaviorTree")
    return trees[0].get("ID") if len(trees) == 1 else None


def build_pyrobosim_tree_from_xml(
    bt_xml: str,
    robot: Any,
    realtime_factor: float = 1.0,
    blackboard: dict[str, Any] | None = None,
) -> py_trees.trees.BehaviourTree:
    """Build an executable tree from BehaviorTree XML.

    Args:
        bt_xml: The tree, as an XML string.
        robot: The PyRoboSim robot whose skills the leaves run.
        realtime_factor: Simulation speed for the robot's actions.
        blackboard: Initial blackboard values, written after the tree is built.
            XML has no place for these; a planner's trees need some.
    """
    main_tree_id = _main_tree_id(bt_xml)
    # The parser reads files, so that <Include> can resolve relative paths.
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "tree.xml")
        with open(path, "w", encoding="utf-8") as f:
            f.write(bt_xml)
        root = parse_behaviour_tree_xml(path, main_tree_id=main_tree_id, node_registry=node_registry(robot, realtime_factor))
    if blackboard:
        seeder = py_trees.blackboard.Client(name="initial_blackboard")
        for key, value in blackboard.items():
            seeder.register_key(key=key, access=py_trees.common.Access.WRITE)
            seeder.set(key, value)
    return py_trees.trees.BehaviourTree(root)


def blackboard_keys(tree: py_trees.trees.BehaviourTree) -> set[str]:
    """Every blackboard key the tree's skills write or its checks read."""
    keys: set[str] = set()
    for node in tree.root.iterate():
        if isinstance(node, RobotActionBehavior):
            keys.update(node.output_keys())
        elif isinstance(node, Check):
            keys.add(node.key if node.key.startswith("/") else f"/{node.key}")
    return keys


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
