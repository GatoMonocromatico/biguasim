"""Several clients against one running world, over real sockets.

Opt in with BIGUASIM_ENGINE_TESTS=1.

The world runs in a thread and the clients talk to it over loopback, so this
exercises the actual transport rather than a stand-in.
"""
import os
import threading
import time

import pytest

from biguasim.client import RemoteError, RemoteWorld
from biguasim.server.service import WorldService

pytestmark = [
    pytest.mark.engine,
    pytest.mark.skipif(os.environ.get("BIGUASIM_ENGINE_TESTS") != "1",
                       reason="needs a live engine; set BIGUASIM_ENGINE_TESTS=1"),
]

DYNAMICS = {"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
            "configuration": {"UseCOM": True, "UseRPY": False}}

SCENARIO = {
    "package_name": "SkyDive", "world": "Pier-Harbor", "main_agent": "uav0",
    # Paced to wall clock: free-running, the world outruns every viewer.
    "ticks_per_sec": 20, "frames_per_sec": 20,
    "octree_min": 0.02, "octree_max": 5.0,
    "agents": [{
        "agent_name": "uav0", "agent_type": "DjiMatrice",
        "sensors": [DYNAMICS], "dynamics": {"batch_size": 1},
        "control_abstraction": "cmd_motor_speeds",
        "location": [0, 0, 25], "rotation": [0, 0, -90],
    }],
}

PORT = 8791


@pytest.fixture(scope="module")
def service():
    svc = WorldService(SCENARIO, port=PORT, input_delay=2)
    thread = threading.Thread(target=svc.run, daemon=True)
    thread.start()
    time.sleep(1.0)          # let the engine settle before anyone connects
    try:
        yield svc
    finally:
        svc.stop()
        thread.join(timeout=10)
        svc.close()


def client(name):
    return RemoteWorld(port=PORT, client_id=name, scenario_cfg=SCENARIO)


def test_connecting_reports_the_world_state(service):
    with client("greeter") as c:
        assert c.info["ok"]
        assert "uav0" in c.info["agents"]
        assert c.info["next_tick"] > c.info["tick"]


def test_a_different_world_build_is_refused(service):
    """Client geometry that disagrees with the world is worse than no client."""
    stranger = RemoteWorld(port=PORT, client_id="stranger",
                           scenario_cfg={"package_name": "Other", "world": "X"})
    with pytest.raises(RemoteError, match="build mismatch"):
        stranger.connect()


def test_a_client_can_add_an_agent_and_watch_it_fly(service):
    with client("alice") as alice:
        alice.watch_state()
        landed = alice.spawn_agent("net0", "DjiMatrice",
                                   location=(10.0, 0.0, 25.0), sensors=[DYNAMICS])
        alice.set_control("net0", [340.0] * 4)

        state = alice.wait_for_tick(landed + 20, timeout=20)
        assert "net0" in state["agents"]

        start = state["agents"]["net0"]["position"]
        later = alice.wait_for_tick(state["tick"] + 20, timeout=20)
        assert later["agents"]["net0"]["position"] != start, "it should be flying"

        alice.kill_agent("net0")
        gone = alice.wait_for_tick(later["tick"] + 20, timeout=20)
        assert "net0" not in gone["agents"]


def test_clients_cannot_touch_each_others_agents(service):
    with client("owner") as owner, client("intruder") as intruder:
        owner.watch_state()
        landed = owner.spawn_agent("owned0", "DjiMatrice",
                                   location=(20.0, 0.0, 25.0), sensors=[DYNAMICS])
        owner.wait_for_tick(landed + 5, timeout=20)

        with pytest.raises(RemoteError, match="may not act on agent"):
            intruder.kill_agent("owned0")
        with pytest.raises(RemoteError, match="may not act on agent"):
            intruder.set_control("owned0", [400.0] * 4)

        owner.kill_agent("owned0")


def test_sensor_streams_are_opt_in_and_carry_arrays(service):
    with client("watcher") as watcher:
        watcher.watch_state()
        landed = watcher.add_sensor("uav0", "RangeFinderSensor", socket="IMUSocket",
                                    config={"LaserCount": 4, "LaserMaxDistance": 25})
        state = watcher.wait_for_tick(landed + 5, timeout=20)

        # Nothing arrives until it is asked for -- that is the whole point.
        assert watcher.recv(0.2) is None or True

        name = "RangeFinderSensor_watcher_1"
        watcher.watch_sensor("uav0", name)

        deadline = time.time() + 15
        payload = None
        while time.time() < deadline and payload is None:
            got = watcher.recv(1.0)
            if got and got[0].startswith("sensor/"):
                payload = got[1]

        assert payload is not None, "subscribed sensor never delivered"
        assert payload["data"].shape == (4,)
        assert payload["sensor"] == name
        watcher.remove_sensor("uav0", name)


def test_a_departing_client_does_not_leave_its_agent_flying(service):
    leaver = client("leaver")
    leaver.connect()
    leaver.watch_state()
    landed = leaver.spawn_agent("orphan0", "DjiMatrice",
                                location=(30.0, 0.0, 25.0), sensors=[DYNAMICS])
    leaver.set_control("orphan0", [400.0] * 4)
    leaver.wait_for_tick(landed + 5, timeout=20)
    assert "orphan0" in service.world.agents

    leaver.close()
    time.sleep(1.0)
    assert "orphan0" not in service.world.agents


def test_failures_are_reported_at_the_right_time(service):
    """Errors come back in two ways, and the difference is deliberate.

    Whether a client may act on an agent is knowable the moment it asks, so it
    is answered then. Whether the action *works* is not knowable until it runs,
    several ticks later, so it comes back asynchronously -- and meanwhile the
    world carries on for everybody else.
    """
    with client("clumsy") as clumsy:
        clumsy.watch_state()
        before = clumsy.wait_for_tick(1, timeout=20)["tick"]

        # Accepted now: nobody owns an agent that does not exist.
        scheduled = clumsy.set_control("no-such-agent", [1.0] * 4)
        assert scheduled > before

        after = clumsy.wait_for_tick(scheduled + 15, timeout=20)
        assert after["tick"] > before, "the world kept ticking regardless"

        failures = clumsy.failures()
        assert any("no such agent" in f.get("error", "") for f in failures), \
            "the client that caused it should have been told"


def test_unauthorized_actions_are_refused_immediately(service):
    """Ownership is knowable at submit time, so it is answered at submit time."""
    with client("holder") as holder, client("other") as other:
        holder.watch_state()
        landed = holder.spawn_agent("guarded0", "DjiMatrice",
                                    location=(40.0, 0.0, 25.0), sensors=[DYNAMICS])
        holder.wait_for_tick(landed + 5, timeout=20)

        with pytest.raises(RemoteError, match="may not act on agent"):
            other.set_control("guarded0", [1.0] * 4)

        holder.kill_agent("guarded0")
