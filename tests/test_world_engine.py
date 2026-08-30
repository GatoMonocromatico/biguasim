"""The world process, against a live engine.

Opt in with BIGUASIM_ENGINE_TESTS=1.

What matters here is that the roster is genuinely mutable while the world runs
-- agents and sensors appearing and disappearing without the simulation
stopping -- and that routing everything through the action queue does not cost
the reproducibility measured in test_determinism.py.
"""
import glob
import os
import subprocess
import sys

import numpy as np
import pytest

from biguasim.server import World
from biguasim.server import actions as act
from biguasim.server.world import RETAINED_BLOCKS_PER_KILLED_AGENT

pytestmark = [
    pytest.mark.engine,
    pytest.mark.skipif(os.environ.get("BIGUASIM_ENGINE_TESTS") != "1",
                       reason="needs a live engine; set BIGUASIM_ENGINE_TESTS=1"),
]

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROBE = os.path.join(REPO, "tools", "determinism_probe.py")

DYNAMICS = {"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
            "configuration": {"UseCOM": True, "UseRPY": False}}

SCENARIO = {
    "package_name": "SkyDive", "world": "Pier-Harbor", "main_agent": "uav0",
    "ticks_per_sec": 20, "frames_per_sec": False,
    "octree_min": 0.02, "octree_max": 5.0,
    "agents": [{
        "agent_name": "uav0", "agent_type": "DjiMatrice",
        "sensors": [DYNAMICS], "dynamics": {"batch_size": 1},
        "control_abstraction": "cmd_motor_speeds",
        "location": [0, 0, 25], "rotation": [0, 0, -90],
    }],
}


@pytest.fixture
def world():
    with World(SCENARIO, input_delay=2) as w:
        yield w


def run_to(world, tick):
    """Advance until the given tick, returning the last state."""
    state = None
    while world.tick < tick:
        state = world.step()
    return state


def test_agents_can_be_added_and_removed_while_running(world):
    assert world.agents == ["uav0"]

    world.submit(act.SpawnAgent(
        client_id="alice", target_tick=world.next_tick, agent="uav9",
        agent_type="DjiMatrice", location=(10.0, 0.0, 25.0), sensors=[DYNAMICS]))
    state = run_to(world, 12)

    assert world.agents == ["uav0", "uav9"]
    assert "uav9" in state, "a spawned agent must appear in the state"
    assert world.owner_of("uav9") == "alice"

    world.submit(act.KillAgent(client_id="alice", target_tick=world.next_tick,
                               agent="uav9"))
    state = run_to(world, 24)

    assert world.agents == ["uav0"]
    assert not world.drain_errors()


def test_sensors_can_be_added_and_removed_while_running(world):
    world.submit(act.AddSensor(
        client_id="bob", target_tick=world.next_tick, agent="uav0",
        sensor_type="RangeFinderSensor", socket="IMUSocket",
        config={"LaserCount": 4, "LaserMaxDistance": 25}))
    state = run_to(world, 10)

    added = [k for k in state["uav0"][0] if k.startswith("RangeFinder")]
    assert len(added) == 1, "the sensor should appear in the state exactly once"

    # It has to keep producing, not just report once. A sensor built with no
    # rate used to go silent after the first tick.
    name = added[0]
    samples = [run_to(world, world.tick + 1)["uav0"][0][name] for _ in range(5)]
    assert all(s is not None for s in samples)
    assert np.any(np.asarray(samples[-1]) != 0)

    world.submit(act.RemoveSensor(client_id="bob", target_tick=world.next_tick,
                                  agent="uav0", sensor_name=name))
    state = run_to(world, world.tick + 6)
    assert name not in state["uav0"][0]


def test_sensor_names_are_assigned_not_taken_from_clients(world):
    """Two clients asking for the same sensor must not collide in /dev/shm."""
    for client in ("alice", "bob"):
        world.submit(act.AddSensor(
            client_id=client, target_tick=world.next_tick, agent="uav0",
            sensor_type="RangeFinderSensor", socket="IMUSocket",
            sensor_name="camera", config={"LaserCount": 4}))
    state = run_to(world, 12)

    added = [k for k in state["uav0"][0] if k != "DynamicsSensor"]
    assert len(added) == 2, "both sensors should exist under distinct names"
    assert len(set(added)) == 2


def test_a_bad_action_does_not_stop_the_world(world):
    """One client's mistake must not take the simulation down."""
    world.submit(act.SetControl(client_id="mallory", target_tick=world.next_tick,
                                agent="does-not-exist", command=[1.0] * 4))
    run_to(world, 8)

    failures = world.drain_errors()
    assert len(failures) == 1 and "no such agent" in failures[0][2]
    assert world.drain_errors() == [], "draining twice must not repeat failures"

    world.submit(act.SetControl(client_id="alice", target_tick=world.next_tick,
                                agent="uav0", command=[330.0] * 4))
    assert run_to(world, 16) is not None


def test_agent_lifecycle_costs_exactly_what_it_should(world):
    """Killing an agent reclaims its sensors but keeps its own five blocks.

    The engine has no despawn, so the actor survives and its buffers stay
    mapped. What matters is that the cost is bounded and predictable rather
    than growing on its own.
    """
    run_to(world, 4)
    blocks = len(glob.glob("/dev/shm/HOLODECK_MEM*"))
    killed = 4

    for i in range(killed):
        world.submit(act.SpawnAgent(
            client_id="alice", target_tick=world.next_tick, agent="tmp%d" % i,
            agent_type="DjiMatrice", location=(5.0 * i, 5.0, 25.0),
            sensors=[DYNAMICS]))
        run_to(world, world.tick + 4)
        world.submit(act.KillAgent(client_id="alice",
                                   target_tick=world.next_tick, agent="tmp%d" % i))
        run_to(world, world.tick + 4)

    assert world.agents == ["uav0"]
    settled = len(glob.glob("/dev/shm/HOLODECK_MEM*"))
    assert settled == blocks + killed * RETAINED_BLOCKS_PER_KILLED_AGENT

    # The point of the accounting: idling must not add to it.
    run_to(world, world.tick + 40)
    assert len(glob.glob("/dev/shm/HOLODECK_MEM*")) == settled


def test_departing_clients_do_not_leave_agents_running(world):
    """A closed laptop must not leave a quadrotor climbing at full power."""
    world.submit(act.SpawnAgent(
        client_id="alice", target_tick=world.next_tick, agent="uav9",
        agent_type="DjiMatrice", location=(10.0, 0.0, 25.0), sensors=[DYNAMICS]))
    world.submit(act.SetControlDefaults(client_id="alice",
                                        target_tick=world.next_tick,
                                        agent="uav9", command=[0.0] * 4))
    world.submit(act.SetControl(client_id="alice", target_tick=world.next_tick,
                                agent="uav9", command=[400.0] * 4))
    run_to(world, 12)

    assert world.release_client("alice") == [], "a default was set, so no kill"
    assert world._controls["uav9"] == [0.0] * 4
    assert world.owner_of("uav9") is None, "ownership is released"


def test_departing_clients_without_defaults_lose_their_agents(world):
    world.submit(act.SpawnAgent(
        client_id="alice", target_tick=world.next_tick, agent="uav9",
        agent_type="DjiMatrice", location=(10.0, 0.0, 25.0), sensors=[DYNAMICS]))
    run_to(world, 10)

    assert world.release_client("alice") == ["uav9"]
    assert world.agents == ["uav0"]


def test_the_world_loop_is_reproducible_across_processes(tmp_path):
    """Everything above, driven by a scripted action stream, twice.

    Out-of-order submissions from two clients, a mid-run spawn, a mid-run
    sensor and a kill -- all of which must land identically both times.
    """
    outs = []
    for run in ("a", "b"):
        out = tmp_path / "world_{}.npz".format(run)
        subprocess.run([sys.executable, PROBE, "--scenario", "world",
                        "--out", str(out)], check=True, cwd=REPO, timeout=600)
        outs.append(np.load(str(out)))

    a, b = outs
    assert set(a.files) == set(b.files), "the same channels must exist both runs"
    assert len(a.files) == 3, "expected uav0 dynamics, uav0 rangefinder, uav9 dynamics"
    for key in a.files:
        assert np.array_equal(a[key], b[key]), "{} diverged".format(key)


def test_an_impossible_spawn_does_not_kill_the_world(world):
    """A handler can raise anything, and none of it may stop the simulation.

    Handlers reach into the environment and the engine, so they raise far more
    than WorldError -- an agent type this build does not have surfaces as a
    KeyError. Catching only WorldError meant any client could kill a world other
    people were using, by accident, with one bad spawn.
    """
    world.submit(act.SpawnAgent(
        client_id="hopeful", target_tick=world.next_tick, agent="bogus",
        agent_type="NoSuchVehicle9000", sensors=[DYNAMICS]))

    before = world.tick
    run_to(world, before + 20)

    failures = world.drain_errors()
    assert len(failures) == 1
    assert "NoSuchVehicle9000" in failures[0][2]
    assert world.tick > before, "the world must have kept ticking"
    assert "bogus" not in world.agents


def test_spawn_time_sensors_honour_their_rate(world):
    """A rate declared at spawn has to mean the same as one added later.

    AddSensor passed Hz through to a tick divider; SpawnAgent silently did not,
    so a camera declared at 10 Hz in a scenario rendered on every tick. At the
    200 ticks/sec an ArduPilot bridge runs at, that is twenty times the
    intended render load -- paid by every client in the world, not just the one
    that asked for it.
    """
    rate = world._env._ticks_per_sec          # 20 in this scenario
    hz = rate / 4                             # every fourth tick

    world.submit(act.SpawnAgent(
        client_id="alice", target_tick=world.next_tick, agent="uav7",
        agent_type="DjiMatrice", location=(20.0, 0.0, 25.0),
        sensors=[DYNAMICS, {"sensor_type": "IMUSensor", "socket": "IMUSocket",
                            "Hz": hz}]))
    run_to(world, world.tick + 5)

    start = world.tick
    seen = 0
    while world.tick < start + 20:
        state = world.step()
        frames = state.get("uav7")
        if frames and "IMUSensor" in frames[0]:
            seen += 1

    # Exactly a divider, not an approximation: 20 ticks at every fourth.
    assert seen == 5, "IMU sampled {} times in 20 ticks, expected 5".format(seen)


def test_a_spawned_agent_answers_its_controls(world):
    """The test that was missing, and the bug it would have caught.

    An agent spawned at runtime got no control scheme, because add_agent does
    not set one and environments.py does it separately, right afterwards, for
    the agents it loads. Every fresh agent starts on scheme 0 while a
    quadrotor's dynamics emit scheme 1, so the engine read thrust as something
    else and the vehicle sat where it landed and answered nothing.

    Nothing raised. The roster was right, the controls dict held the right
    numbers, and the vehicle simply never moved -- so the assertion has to be
    about motion, not about bookkeeping.
    """
    world.submit(act.SpawnAgent(
        client_id="alice", target_tick=world.next_tick, agent="uav5",
        agent_type="DjiMatrice", location=(30.0, 0.0, 25.0), sensors=[DYNAMICS]))
    state = run_to(world, world.tick + 10)

    engine_agent = world._env.agents["uav5-id0"]
    model = world._env._dynamics_dict["uav5"]
    assert engine_agent._current_control_scheme == model._scheme, \
        "the engine will read this agent's actions under the wrong scheme"

    def altitude(state):
        return float(np.asarray(state["uav5"][0]["DynamicsSensor"])[8])

    before = altitude(state)
    world.submit(act.SetControl(client_id="alice", target_tick=world.next_tick,
                                agent="uav5", command=[600.0] * 4))
    for _ in range(120):
        state = world.step()

    assert altitude(state) > before + 1.0, \
        "full throttle moved it {:.2f} m".format(altitude(state) - before)
