"""Burying a puppet whose original has gone.

These need no engine: the retirement path is about bookkeeping, and the one
thing worth pinning down is that the viewer keeps asking until the actor has
actually moved. A park issued once and assumed to have worked leaves a vehicle
drawn in a world that no longer contains it, and nothing ever corrects it --
even though the world goes on saying the agent is gone every tick.
"""
import numpy as np

from biguasim.client.viewer import GRAVEYARD, PARK_ATTEMPTS, Viewer


class FakeSensor:
    def __init__(self, z):
        self.sensor_data = np.zeros(19, dtype=np.float32)
        self.sensor_data[8] = z


class FakeAgent:
    """An actor that reports where it is, and may refuse to be moved."""

    def __init__(self, z=5.0, movable=True, reports=True):
        self.movable = movable
        self.sensors = {"DynamicsSensor": FakeSensor(z)} if reports else {}
        self.teleports = 0

    def set_physics_state(self, location, rotation, velocity, angular_velocity):
        self.teleports += 1
        if not self.movable:
            # What Unreal does with an out-of-bounds location: nothing at all,
            # and no error either.
            return
        sensor = self.sensors.get("DynamicsSensor")
        if sensor is not None:
            sensor.sensor_data[8] = location[2]


class FakeEnv:
    def __init__(self, agents=None):
        self.agents = dict(agents or {})


def viewer_with(agents):
    """A Viewer with only the fields the retirement path touches."""
    viewer = object.__new__(Viewer)
    viewer._env = FakeEnv(agents)
    viewer._puppets = {}
    viewer._parked = {}
    return viewer


def test_a_departed_puppet_is_parked_then_left_alone():
    agent = FakeAgent(z=5.0)
    viewer = viewer_with({"uav0-id0": agent})
    viewer._puppets["uav0"] = "DjiMatrice"

    viewer._retire("uav0")
    assert viewer.puppets == {}, "it must stop being drawn immediately"

    viewer._park_departed()
    assert agent.sensors["DynamicsSensor"].sensor_data[8] == GRAVEYARD[2]
    assert "uav0" in viewer._parked, "not confirmed until the next frame reads it"

    viewer._park_departed()
    assert "uav0" not in viewer._parked, "confirmed arrived, so stop asking"

    before = agent.teleports
    viewer._park_departed()
    assert agent.teleports == before, "a buried puppet costs nothing per frame"


def test_an_actor_created_after_retirement_is_still_parked():
    """The race the old one-shot park lost silently.

    An agent whose whole life in the stream is shorter than the local engine
    takes to create its actor was retired while there was nothing to move, and
    the name was dropped anyway -- so the actor turned up with nobody left
    tracking it, and stayed where it was spawned for the rest of the session.
    """
    viewer = viewer_with({})
    viewer._puppets["uav0"] = "DjiMatrice"
    viewer._retire("uav0")

    for _ in range(3):
        viewer._park_departed()          # no actor yet; must not give up
    assert "uav0" in viewer._parked

    agent = FakeAgent(z=5.0)
    viewer._env.agents["uav0-id0"] = agent
    viewer._park_departed()
    assert agent.sensors["DynamicsSensor"].sensor_data[8] == GRAVEYARD[2]


def test_a_park_the_engine_ignores_is_reported(capsys):
    """Repeating a teleport does not surface one that will never be honoured."""
    agent = FakeAgent(z=5.0, movable=False)
    viewer = viewer_with({"uav0-id0": agent})
    viewer._puppets["uav0"] = "DjiMatrice"
    viewer._retire("uav0")

    for _ in range(PARK_ATTEMPTS + 1):
        viewer._park_departed()

    assert "uav0" not in viewer._parked, "gives up rather than retrying forever"
    warning = capsys.readouterr().out
    assert "uav0" in warning and "world bounds" in warning


def test_a_puppet_that_cannot_say_where_it_is_reports_differently(capsys):
    """Unverifiable is not the same answer as stuck, and must not read like it."""
    agent = FakeAgent(z=5.0, reports=False)
    viewer = viewer_with({"uav0-id0": agent})
    viewer._puppets["uav0"] = "DjiMatrice"
    viewer._retire("uav0")

    for _ in range(PARK_ATTEMPTS + 1):
        viewer._park_departed()

    warning = capsys.readouterr().out
    assert "never reported where it went" in warning
