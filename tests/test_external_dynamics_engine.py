"""Vehicles the world does not integrate.

Opt in with BIGUASIM_ENGINE_TESTS=1.

Option (b): the client works out where its vehicle is -- a custom dynamics
model, a hardware rig, a real airframe -- and the world does collision and
sensors for it, and decides what everyone else sees.
"""
import os
import sys

import numpy as np
import pytest

from biguasim.server import World, WorldError
from biguasim.server import actions as act

pytestmark = [
    pytest.mark.engine,
    pytest.mark.skipif(os.environ.get("BIGUASIM_ENGINE_TESTS") != "1",
                       reason="needs a live engine; set BIGUASIM_ENGINE_TESTS=1"),
]

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

DYNAMICS = {"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
            "configuration": {"UseCOM": True, "UseRPY": False}}
COLLISION = {"sensor_type": "CollisionSensor", "socket": "IMUSocket"}


@pytest.fixture
def world():
    from determinism_probe import build
    cfg, _names, _ticks, _command = build("open")
    with World(cfg, input_delay=2) as w:
        w.submit(act.SpawnAgent(
            client_id="hil", target_tick=w.next_tick, agent="rig",
            agent_type="DjiMatrice", location=(0.0, 0.0, 60.0),
            sensors=[DYNAMICS, COLLISION], externally_driven=True))
        while w.tick < 6:
            w.step()
        yield w


def run_to(world, tick):
    state = None
    while world.tick < tick:
        state = world.step()
    return state


def position(state, agent):
    return np.asarray(state[agent][0]["DynamicsSensor"], dtype=np.float64)[6:9]


def test_the_world_does_not_integrate_a_client_driven_agent(world):
    assert "rig" in world._external
    assert "rig" not in world._env._dynamics_dict


def test_control_commands_are_refused_with_a_useful_message(world):
    world.submit(act.SetControl(client_id="hil", target_tick=world.next_tick,
                                agent="rig", command=[300.0] * 4))
    run_to(world, world.tick + 6)

    failures = world.drain_errors()
    assert len(failures) == 1
    assert "use set_pose" in failures[0][2]


def test_the_client_decides_where_it_is(world):
    state = None
    for step in range(30):
        world.submit(act.SetPose(
            client_id="hil", target_tick=world.next_tick, agent="rig",
            position=(0.0, 0.0, 60.0 - step), velocity=(0.0, 0.0, -20.0)))
        state = world.step()

    # It went where it was put, not where physics would have taken it.
    assert position(state, "rig")[2] == pytest.approx(60.0 - 29, abs=3.0)
    assert not world.drain_errors()


def test_the_world_still_owns_collision(world):
    """The client can ask to be inside the terrain. The world says otherwise."""
    corrections = []
    z = 60.0
    for _ in range(200):
        z -= 0.8
        world.submit(act.SetPose(client_id="hil", target_tick=world.next_tick,
                                 agent="rig", position=(0.0, 0.0, z),
                                 velocity=(0.0, 0.0, -16.0)))
        world.step()
        for tick, agent, pose in world.drain_corrections():
            corrections.append((tick, agent, pose))
            z = pose["position"][2]      # this client chooses to accept them

    assert corrections, "flying into the ground should be corrected"
    assert all(agent == "rig" for _, agent, _ in corrections)

    settled = corrections[-1][2]["position"][2]
    assert settled > -85.0, "the world should have stopped it at the surface"


def test_sensors_still_work_for_client_driven_agents(world):
    """The world simulates its sensors even though it does not fly it."""
    state = run_to(world, world.tick + 5)
    assert state["rig"][0]["DynamicsSensor"] is not None
    assert state["rig"][0]["CollisionSensor"] is not None
