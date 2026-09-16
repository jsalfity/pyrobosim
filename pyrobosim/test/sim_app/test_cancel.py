#!/usr/bin/env python3

"""
Stopping behavior-tree runs in the sim_app server.

A run that is abandoned (server deadline, tick budget, /cancel_bt, or a new
start/reset/reload) must stop before anything else happens: its ticking thread
and every action worker it started are joined, so an old tree can no longer
move the robot, re-parent objects in the reset world, or write the
blackboard. These tests drive SimContext in-process, without the GUI; one test
also goes through the HTTP handler on an ephemeral localhost port.
"""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from typing import Any, Callable, Iterator

import py_trees
import pytest

from pyrobosim.behaviors import RobotActionBehavior
from pyrobosim.core.robot import Robot
from pyrobosim.planning.actions import ExecutionStatus, TaskAction
from pyrobosim.sim_app import server

WORLD_FILE = "roscon_2024_workshop_world.yaml"


#####################
# Tree construction #
#####################
def cond(key: str, op: str, value: Any, name: str | None = None) -> dict[str, Any]:
    return {
        "type": "condition",
        "name": name or f"{key}{op}{value}",
        "key": key,
        "operator": op,
        "value": value,
    }


def act(
    name: str, action: str, params: dict[str, Any], outputs: dict[str, str]
) -> dict[str, Any]:
    return {
        "type": "action",
        "name": name,
        "action": action,
        "params": params,
        "outputs": outputs,
    }


def seq(*children: dict[str, Any], memory: bool = False) -> dict[str, Any]:
    return {"type": "sequence", "memory": memory, "children": list(children)}


# Policy-shaped tree whose goal never holds: navigate desk <-> pantry forever.
# Its keys carry a LEAK_ prefix, so any later appearance is attributable.
NAV_OUT = {"robot_location": "LEAK_loc"}
NAV_LIVELOCK = {
    "blackboard": {"initial": {"LEAK_loc": "dining"}},
    "root": {
        "type": "selector",
        "memory": False,
        "children": [
            cond("LEAK_loc", "==", "__never__", "goal_never_holds"),
            seq(
                cond("LEAK_loc", "==", "desk"),
                act("nav_pantry", "navigate", {"target_location": "pantry_storage"}, NAV_OUT),
            ),
            act("nav_desk", "navigate", {"target_location": "desk_desktop"}, NAV_OUT),
        ],
    },
}

# Plan-shaped task: go to the pantry and pick the bread.
TRIVIAL_PLAN = {
    "root": seq(
        act("nav_pantry", "navigate", {"target_location": "pantry_storage"}, {"robot_location": "robot_location"}),
        act("pick_bread", "pick", {"object": "bread"}, {"held_object": "held_object"}),
        memory=True,
    )
}

PICK_WASTE = {
    "root": seq(
        act("nav_desk", "navigate", {"target_location": "desk_desktop"}, {}),
        act("pick_waste", "pick", {"object": "waste0"}, {"held_object": "LEAK_held"}),
        memory=True,
    )
}

PLACE_WASTE = {
    "root": seq(
        act("nav_desk", "navigate", {"target_location": "desk_desktop"}, {}),
        act("pick_waste", "pick", {"object": "waste0"}, {}),
        act("nav_table", "navigate", {"target_location": "table_tabletop"}, {}),
        act("place_waste", "place", {}, {"objects_here": "LEAK_here"}),
        memory=True,
    )
}

# Banked failure modes that used to hang the executor: pick with no object
# (UnboundLocalError inside Robot.pick_object) and place with a dict pose
# (AttributeError inside Robot.place_object).
PICK_NONE = {
    "root": seq(
        act("nav_desk", "navigate", {"target_location": "desk_desktop"}, {}),
        act("pick_nothing", "pick", {}, {"status": "pick_status"}),
        memory=True,
    )
}
PLACE_DICT_POSE = {
    "root": seq(
        act("nav_desk", "navigate", {"target_location": "desk_desktop"}, {}),
        act("pick_waste", "pick", {"object": "waste0"}, {}),
        act("place_dict_pose", "place", {"pose": {"x": 1.0, "y": 2.0}}, {"status": "place_status"}),
        memory=True,
    )
}


###########
# Helpers #
###########
def wait_until(predicate: Callable[[], bool], timeout_s: float, step_s: float = 0.01) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(step_s)
    return predicate()


def wait_final(ctx: server.SimContext, run_id: str, timeout_s: float) -> dict[str, Any]:
    wait_until(lambda: ctx.bt_status(run_id)["status"] != "RUNNING", timeout_s, 0.05)
    return ctx.bt_status(run_id)


def object_locations(ctx: server.SimContext) -> dict[str, str | None]:
    return {obj["name"]: obj["location"] for obj in ctx.world_state()["objects"]}


def leak_keys() -> dict[str, Any]:
    return {
        key: value
        for key, value in py_trees.blackboard.Blackboard.storage.items()
        if "LEAK" in key
    }


def run_threads(ctx: server.SimContext, run_id: str) -> tuple[threading.Thread, list[RobotActionBehavior]]:
    entry = ctx.bt_runs[run_id]
    return entry["thread"], list(entry.get("actions") or [])


def slow_down(monkeypatch: pytest.MonkeyPatch, method: str, delay_s: float) -> None:
    """Make Robot.<method> take delay_s, so the action is observably in flight."""
    original = getattr(Robot, method)

    def slow(self: Robot, *args: Any, **kwargs: Any) -> Any:
        time.sleep(delay_s)
        return original(self, *args, **kwargs)

    monkeypatch.setattr(Robot, method, slow)


############
# Fixtures #
############
@pytest.fixture(scope="module")
def ctx() -> Iterator[server.SimContext]:
    world, world_path = server.load_world(WORLD_FILE)
    context = server.SimContext(world, world_path)
    context.reset_world(deterministic=True)
    yield context
    context.cancel_bt(None)
    context.world.shutdown()


@pytest.fixture(autouse=True)
def fresh_world(ctx: server.SimContext) -> Iterator[None]:
    ctx.reset_world(deterministic=True)
    py_trees.blackboard.Blackboard.clear()
    yield
    ctx.cancel_bt(None)
    assert ctx.live_runs()["live_runs"] == 0


#########
# Tests #
#########
def test_plan_tree_that_finishes_is_unaffected(ctx: server.SimContext) -> None:
    """A tree that ends before its deadline ends exactly as it did without one."""
    finals = []
    for timeout_s in (0.0, 30.0):
        ctx.reset_world(deterministic=True)
        run_id = ctx.start_bt(TRIVIAL_PLAN, None, 100, realtime_factor=5.0, print_every=0, timeout_s=timeout_s)
        status = wait_final(ctx, run_id, 30.0)
        state = ctx.world_state()
        finals.append((status["status"], status.get("failed_node"), state["held_object"], state["robot_location"]))
        assert status["halted"] is True
        assert "timeout_source" not in status
        assert ctx.live_runs()["live_runs"] == 0
    assert finals[0] == finals[1]
    assert finals[0][0] == "SUCCESS"
    assert finals[0][2]["category"] == "bread"


def test_cancel_while_navigating(ctx: server.SimContext) -> None:
    run_id = ctx.start_bt(NAV_LIVELOCK, None, 100, realtime_factor=1.0, print_every=0, policy_mode=True)
    robot = ctx.get_robot()
    runner, actions = run_threads(ctx, run_id)
    assert wait_until(lambda: robot.is_busy() and robot.path_executor.following_path, 15.0)

    start = time.monotonic()
    outcome = ctx.cancel_bt(run_id)
    elapsed = time.monotonic() - start

    assert outcome["alive"] is False and outcome["halted"] is True
    assert outcome["status"] == "CANCELED"
    assert elapsed < 1.0, elapsed
    assert not runner.is_alive()
    assert all(not node.live_workers() for node in actions)
    assert not robot.is_busy()
    pose = robot.get_pose()
    time.sleep(0.5)
    assert robot.get_pose().is_approx(pose)
    assert ctx.bt_status(run_id)["halted"] is True


def test_stop_cancels_navigation_left_behind_by_a_dead_runner(
    ctx: server.SimContext, monkeypatch: pytest.MonkeyPatch
) -> None:
    """If the ticking thread gave up before its navigation stopped, the stop finishes the job."""
    monkeypatch.setattr(server, "HALT_LIMIT_S", 0.0)
    monkeypatch.setattr(RobotActionBehavior, "terminate", lambda self, new_status: None)
    real_cancel_navigation = server.cancel_navigation
    monkeypatch.setattr(server, "cancel_navigation", lambda robot: None)
    run_id = ctx.start_bt(NAV_LIVELOCK, None, 100, realtime_factor=1.0, print_every=0, policy_mode=True)
    robot = ctx.get_robot()
    runner, actions = run_threads(ctx, run_id)
    assert wait_until(lambda: robot.path_executor.following_path, 15.0)
    ctx.bt_runs[run_id]["cancel_event"].set()
    assert wait_until(lambda: not runner.is_alive(), 2.0)
    assert ctx.bt_status(run_id)["halted"] is False  # its own halt had no time
    assert robot.path_executor.following_path  # still driving

    monkeypatch.setattr(server, "cancel_navigation", real_cancel_navigation)
    start = time.monotonic()
    outcome = ctx.cancel_bt(run_id)
    assert time.monotonic() - start < 1.0
    assert outcome["alive"] is False and outcome["halted"] is True
    assert all(not node.live_workers() for node in actions)
    assert not robot.is_busy()


@pytest.mark.parametrize("tree, method", [(PICK_WASTE, "pick_object"), (PLACE_WASTE, "place_object")])
def test_cancel_while_manipulating(
    ctx: server.SimContext, monkeypatch: pytest.MonkeyPatch, tree: dict[str, Any], method: str
) -> None:
    """Pick and place cannot be interrupted: the stop waits for them, then nothing is written."""
    delay_s = 0.5
    slow_down(monkeypatch, method, delay_s)
    action_type = method.split("_")[0]
    run_id = ctx.start_bt(tree, None, 100, realtime_factor=5.0, print_every=0)
    robot = ctx.get_robot()
    runner, actions = run_threads(ctx, run_id)
    assert wait_until(
        lambda: robot.is_busy() and getattr(robot.current_action, "type", None) == action_type,
        30.0,
    )

    start = time.monotonic()
    outcome = ctx.cancel_bt(run_id)
    elapsed = time.monotonic() - start

    assert outcome["alive"] is False and outcome["halted"] is True
    assert outcome["status"] == "CANCELED"
    assert elapsed < delay_s + 1.0, elapsed
    assert not runner.is_alive()
    assert all(not node.live_workers() for node in actions)
    assert not robot.is_busy()
    # The action finished inside its worker, but its node was never ticked
    # again, so its outputs never reached the blackboard.
    assert leak_keys() == {}


def test_reset_during_manipulation_is_prompt_and_clean(
    ctx: server.SimContext, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Without the stop, a reset racing a place hung in World.remove_location."""
    initial = object_locations(ctx)
    slow_down(monkeypatch, "place_object", 0.5)
    run_id = ctx.start_bt(PLACE_WASTE, None, 100, realtime_factor=5.0, print_every=0)
    old_robot = ctx.get_robot()
    runner, actions = run_threads(ctx, run_id)
    assert wait_until(lambda: getattr(old_robot.current_action, "type", None) == "place", 30.0)

    start = time.monotonic()
    assert ctx.reset_world(deterministic=True) is True
    elapsed = time.monotonic() - start

    assert elapsed < 1.5, elapsed
    assert not runner.is_alive()
    assert all(not node.live_workers() for node in actions)
    assert ctx.bt_status(run_id)["status"] == "CANCELED"
    assert ctx.get_robot() is not old_robot
    assert old_robot.world is None
    assert object_locations(ctx) == initial
    assert ctx.world_state()["held_object"] is None
    time.sleep(1.0)
    assert object_locations(ctx) == initial


def test_server_deadline_reports_timeout_after_stopping(ctx: server.SimContext) -> None:
    timeout_s = 3.0
    start = time.monotonic()
    run_id = ctx.start_bt(
        NAV_LIVELOCK, None, 100, realtime_factor=1.0, print_every=0, policy_mode=True, timeout_s=timeout_s
    )
    runner, actions = run_threads(ctx, run_id)
    status = wait_final(ctx, run_id, timeout_s + server.HALT_LIMIT_S + 2.0)
    elapsed = time.monotonic() - start

    assert status["status"] == "TIMEOUT"
    assert status["timeout_source"] == "server_deadline"
    assert "failed_node" not in status
    assert status["halted"] is True
    assert timeout_s <= elapsed < timeout_s + 2.0, elapsed
    # TIMEOUT is published only after the tree has been stopped.
    assert not any(node.live_workers() for node in actions)
    assert not ctx.get_robot().is_busy()
    assert wait_until(lambda: not runner.is_alive(), 1.0)
    assert ctx.live_runs()["live_runs"] == 0


def test_tick_budget_timeout_also_stops_the_tree(ctx: server.SimContext) -> None:
    run_id = ctx.start_bt(NAV_LIVELOCK, None, 100, realtime_factor=1.0, print_every=0, policy_mode=True, max_ticks=15)
    _, actions = run_threads(ctx, run_id)
    status = wait_final(ctx, run_id, 20.0)
    assert status["status"] == "TIMEOUT"
    assert status["failed_node"] == "tick_budget_exhausted"
    assert status["halted"] is True
    assert not any(node.live_workers() for node in actions)
    assert not ctx.get_robot().is_busy()


def test_old_tree_never_writes_after_next_start(ctx: server.SimContext) -> None:
    run_id = ctx.start_bt(NAV_LIVELOCK, None, 100, realtime_factor=1.0, print_every=0, policy_mode=True)
    robot = ctx.get_robot()
    old_runner, old_actions = run_threads(ctx, run_id)
    assert wait_until(lambda: robot.path_executor.following_path, 15.0)

    writes_after_stop: list[str] = []
    stopped = threading.Event()
    for node in old_actions:
        original = node._write_outputs

        def counting(result: Any, _node: RobotActionBehavior = node, _orig: Any = original) -> None:
            if stopped.is_set():
                writes_after_stop.append(_node.name)
            _orig(result)

        node._write_outputs = counting  # type: ignore[method-assign]

    start = time.monotonic()
    new_id = ctx.start_bt(TRIVIAL_PLAN, None, 100, realtime_factor=5.0, print_every=0, timeout_s=60.0)
    stopped.set()
    assert time.monotonic() - start < 1.0
    assert ctx.bt_status(run_id)["status"] == "CANCELED"
    assert not old_runner.is_alive()
    assert all(not node.live_workers() for node in old_actions)

    waste_before = {k: v for k, v in object_locations(ctx).items() if k.startswith("waste")}
    watch_end = time.monotonic() + 20.0
    while time.monotonic() < watch_end:
        assert leak_keys() == {}
        assert writes_after_stop == []
        time.sleep(0.25)
    assert wait_final(ctx, new_id, 1.0)["status"] == "SUCCESS"
    assert {k: v for k, v in object_locations(ctx).items() if k.startswith("waste")} == waste_before


@pytest.mark.parametrize(
    "tree, node_name, status_key",
    [(PICK_NONE, "pick_nothing", "/pick_status"), (PLACE_DICT_POSE, "place_dict_pose", "/place_status")],
)
def test_action_that_raises_fails_instead_of_hanging(
    ctx: server.SimContext, tree: dict[str, Any], node_name: str, status_key: str
) -> None:
    run_id = ctx.start_bt(tree, None, 100, realtime_factor=5.0, print_every=0, timeout_s=60.0)
    status = wait_final(ctx, run_id, 20.0)
    robot = ctx.get_robot()
    assert status["status"] == "FAILURE"
    assert status["failed_node"] == node_name
    assert status["blackboard"][status_key] == "EXECUTION_FAILURE"
    assert status["halted"] is True
    assert not robot.executing_action and robot.current_action is None and not robot.is_busy()
    assert ctx.live_runs()["live_runs"] == 0
    # The robot is usable afterwards.
    ctx.reset_world(deterministic=True)
    run_id = ctx.start_bt(TRIVIAL_PLAN, None, 100, realtime_factor=5.0, print_every=0, timeout_s=60.0)
    assert wait_final(ctx, run_id, 30.0)["status"] == "SUCCESS"


def test_start_and_reset_refused_while_a_run_will_not_stop(
    ctx: server.SimContext, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(server, "HALT_LIMIT_S", 0.2)
    monkeypatch.setattr(server, "CANCEL_JOIN_S", 0.5)
    slow_down(monkeypatch, "pick_object", 3.0)
    run_id = ctx.start_bt(PICK_WASTE, None, 100, realtime_factor=5.0, print_every=0)
    robot = ctx.get_robot()
    assert wait_until(lambda: getattr(robot.current_action, "type", None) == "pick", 30.0)

    with pytest.raises(server.RunStillAliveError):
        ctx.reset_world(deterministic=True)
    assert ctx.live_runs()["live_runs"] == 1
    assert ctx.bt_status(run_id)["halted"] is False
    with pytest.raises(server.RunStillAliveError):
        ctx.start_bt(TRIVIAL_PLAN, None, 100, realtime_factor=5.0, print_every=0)

    # Once the pick has returned, the same requests go through.
    monkeypatch.setattr(server, "CANCEL_JOIN_S", 10.0)
    assert ctx.reset_world(deterministic=True) is True
    assert ctx.bt_status(run_id)["halted"] is True
    assert ctx.live_runs()["live_runs"] == 0


def test_http_endpoints(ctx: server.SimContext, monkeypatch: pytest.MonkeyPatch) -> None:
    httpd = server.start_control_server(ctx, "127.0.0.1", 0)
    base = f"http://127.0.0.1:{httpd.server_address[1]}"

    def post(path: str, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        request = urllib.request.Request(
            base + path,
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read())

    try:
        with urllib.request.urlopen(base + "/live_runs", timeout=5) as response:
            assert json.loads(response.read())["live_runs"] == 0

        code, body = post(
            "/run_bt",
            {"bt_json": NAV_LIVELOCK, "tick_ms": 100, "realtime_factor": 1.0, "print_every": 0,
             "policy_mode": True, "timeout_s": 30},
        )
        assert code == 200
        run_id = body["run_id"]
        assert ctx.bt_runs[run_id]["timeout_s"] == 30.0
        assert post("/live_runs", {})[1]["live_runs"] == 1
        assert post("/cancel_bt", {})[0] == 400
        code, body = post("/cancel_bt", {"run_id": run_id})
        assert code == 200 and body["alive"] is False and body["halted"] is True
        assert post("/bt_status", {"run_id": run_id})[1]["status"] == "CANCELED"
        assert post("/live_runs", {})[1]["live_runs"] == 0

        # A run that will not stop in time: start and reset are refused with 409.
        monkeypatch.setattr(server, "HALT_LIMIT_S", 0.2)
        monkeypatch.setattr(server, "CANCEL_JOIN_S", 0.5)
        slow_down(monkeypatch, "pick_object", 3.0)
        code, body = post("/run_bt", {"bt_json": PICK_WASTE, "realtime_factor": 5.0, "print_every": 0})
        assert code == 200
        robot = ctx.get_robot()
        assert wait_until(lambda: getattr(robot.current_action, "type", None) == "pick", 30.0)
        code, body = post("/reset_world", {"deterministic": True})
        assert code == 409 and body["live_runs"] == 1
        code, body = post("/run_bt", {"bt_json": TRIVIAL_PLAN, "print_every": 0})
        assert code == 409
        code, body = post("/cancel_bt", {"all": True})
        assert code == 200 and body["still_alive"] == 1

        monkeypatch.setattr(server, "CANCEL_JOIN_S", 10.0)
        code, body = post("/reset_world", {"deterministic": True})
        assert code == 200 and body == {"reset": True}
        assert post("/live_runs", {})[1]["live_runs"] == 0
    finally:
        httpd.shutdown()
        httpd.server_close()


class _FakeExecutor:
    def __init__(self) -> None:
        self.cancel_execution = False


class _FakeRobot:
    """Just enough of Robot for RobotActionBehavior, with a scripted execute_action."""

    def __init__(self, execute: Callable[["_FakeRobot", TaskAction], Any]) -> None:
        self.execute = execute
        self.executing_action = False
        self.executing_nav = False
        self.executing_plan = False
        self.current_action: TaskAction | None = None
        self.path_executor = _FakeExecutor()
        self.location = None
        self.manipulated_object = None

    def is_busy(self) -> bool:
        return self.executing_action or self.executing_plan or self.executing_nav

    def execute_action(self, action: TaskAction, realtime_factor: float = 1.0) -> Any:
        # Mirrors Robot.execute_action: flags set on entry, cleared only on return.
        self.executing_action = True
        self.current_action = action
        if action.type == "navigate":
            self.executing_nav = True
        result = self.execute(self, action)
        self.executing_nav = False
        self.current_action = None
        self.executing_action = False
        return result


def _tick_until_done(node: RobotActionBehavior, timeout_s: float = 5.0) -> py_trees.common.Status:
    end = time.monotonic() + timeout_s
    node.tick_once()
    while node.status == py_trees.common.Status.RUNNING and time.monotonic() < end:
        time.sleep(0.01)
        node.tick_once()
    return node.status


@pytest.mark.parametrize("action_type", ["pick", "navigate"])
def test_behavior_releases_flags_when_action_raises(action_type: str) -> None:
    def boom(robot: _FakeRobot, action: TaskAction) -> Any:
        raise UnboundLocalError("local variable 'obj' referenced before assignment")

    robot = _FakeRobot(boom)
    node = RobotActionBehavior("raiser", robot, action_type, outputs={"status": "raiser_status"})
    assert _tick_until_done(node) == py_trees.common.Status.FAILURE
    assert not robot.executing_action and not robot.executing_nav and robot.current_action is None
    assert py_trees.blackboard.Blackboard.storage["/raiser_status"] == ExecutionStatus.EXECUTION_FAILURE.name
    assert not node.live_workers()


def test_behavior_leaves_flags_it_does_not_own() -> None:
    """The finally only clears flags that the raising call itself set."""
    other = TaskAction(type="pick")

    def boom(robot: _FakeRobot, action: TaskAction) -> Any:
        robot.current_action = other  # a concurrent action took over the flags
        raise AttributeError("'dict' object has no attribute 'x'")

    robot = _FakeRobot(boom)
    node = RobotActionBehavior("raiser", robot, "place")
    assert _tick_until_done(node) == py_trees.common.Status.FAILURE
    assert robot.executing_action is True and robot.current_action is other


def test_interrupt_without_halt_request_keeps_old_behaviour() -> None:
    """A priority interrupt in a normal run does not cancel the action; a requested halt does."""
    release = threading.Event()

    def navigate_until_released(robot: _FakeRobot, action: TaskAction) -> Any:
        while not release.is_set() and not robot.path_executor.cancel_execution:
            time.sleep(0.01)
        from pyrobosim.planning.actions import ExecutionResult

        status = ExecutionStatus.CANCELED if robot.path_executor.cancel_execution else ExecutionStatus.SUCCESS
        return ExecutionResult(status=status)

    robot = _FakeRobot(navigate_until_released)
    node = RobotActionBehavior("nav", robot, "navigate", params={"target_location": "desk"})
    node.tick_once()
    assert wait_until(lambda: robot.executing_nav, 2.0)

    node.stop(py_trees.common.Status.INVALID)  # what a selector does on preemption
    time.sleep(0.2)
    assert robot.path_executor.cancel_execution is False
    assert node.live_workers()
    release.set()
    assert wait_until(lambda: not node.live_workers(), 2.0)

    release.clear()
    node.tick_once()
    assert wait_until(lambda: robot.executing_nav, 2.0)
    node.request_halt(time.monotonic() + 2.0)
    start = time.monotonic()
    node.stop(py_trees.common.Status.INVALID)
    assert time.monotonic() - start < 1.0
    assert robot.path_executor.cancel_execution is True
    assert not node.live_workers()
