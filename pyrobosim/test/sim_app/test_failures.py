#!/usr/bin/env python3

"""Skill failure profiles in the sim_app server (SimContext._apply_failures)."""

from __future__ import annotations

import json
import urllib.request
from pathlib import Path
from typing import Iterator

import pytest

from pyrobosim.sim_app import server

WORLD_FILE = "roscon_2024_workshop_world.yaml"
SKILLS = ("close", "detect", "navigate", "open", "pick", "place")


@pytest.fixture(scope="module")
def ctx() -> Iterator[server.SimContext]:
    world, world_path = server.load_world(WORLD_FILE)
    profile, failures = server.load_failure_profile("nominal")
    context = server.SimContext(world, world_path, failures=failures, failure_profile=profile)
    yield context
    context.world.shutdown()


def draws(ctx: server.SimContext, n: int = 30) -> dict[str, list[float]]:
    options = ctx.get_robot().action_execution_options
    return {skill: [float(options[skill].rng.random()) for _ in range(n)] for skill in SKILLS}


def test_profiles_load_in_pyrobosims_own_format() -> None:
    for name, pick in (("nominal", 0.90), ("harsh", 0.75)):
        loaded, options = server.load_failure_profile(name)
        assert loaded == name
        assert set(options) == set(SKILLS)
        assert options["pick"]["success_probability"] == pick


def test_profile_with_an_unknown_argument_is_refused(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text("action_execution_options:\n  pick: {success_prob: 0.9}\n")
    with pytest.raises(TypeError):
        server.load_failure_profile(path)


def test_robot_gets_the_profile(ctx: server.SimContext) -> None:
    options = ctx.get_robot().action_execution_options
    assert {skill: options[skill].success_probability for skill in SKILLS} == {
        "close": 0.95, "detect": 0.95, "navigate": 0.95, "open": 0.95, "pick": 0.90, "place": 0.95,
    }


def test_same_seed_same_failures_other_seed_other_failures(ctx: server.SimContext) -> None:
    ctx.reset_world(deterministic=True, seed=7)
    first = draws(ctx)
    ctx.reset_world(deterministic=True, seed=7)
    assert draws(ctx) == first
    ctx.reset_world(deterministic=True, seed=8)
    assert draws(ctx) != first
    # Each skill has its own stream.
    assert len({tuple(seq) for seq in first.values()}) == len(SKILLS)
    assert ctx.failure_info()["seed"] == 8


def test_reset_without_seed_uses_the_profiles_seeds(ctx: server.SimContext) -> None:
    from pyrobosim.planning.actions import ExecutionOptions

    ctx.reset_world(deterministic=True)
    expected = ExecutionOptions(**ctx.failures["pick"])
    got = ctx.get_robot().action_execution_options["pick"]
    assert [got.rng.random() for _ in range(10)] == [expected.rng.random() for _ in range(10)]
    assert ctx.failure_info()["seed"] is None


def test_reload_reapplies_the_profile(ctx: server.SimContext) -> None:
    replaced = ctx.world
    ctx.reload_world()
    replaced.shutdown()  # reload leaves the old world's sensor threads running
    assert ctx.get_robot().action_execution_options["pick"].success_probability == 0.90


def test_failures_endpoint(ctx: server.SimContext) -> None:
    ctx.reset_world(deterministic=True, seed=3)
    httpd = server.start_control_server(ctx, "127.0.0.1", 0)
    try:
        url = f"http://127.0.0.1:{httpd.server_address[1]}/failures"
        with urllib.request.urlopen(url, timeout=5) as response:
            body = json.loads(response.read())
        assert body["profile"] == "nominal" and body["seed"] == 3
        assert body["action_execution_options"]["pick"]["success_probability"] == 0.90
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_no_profile_leaves_robots_alone() -> None:
    world, world_path = server.load_world(WORLD_FILE)
    try:
        context = server.SimContext(world, world_path)
        assert context.get_robot().action_execution_options == {}
        assert context.failure_info()["profile"] is None
    finally:
        world.shutdown()
