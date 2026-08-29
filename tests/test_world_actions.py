"""Action encoding, ordering and ownership.

No engine: these cover the parts of the world that decide *what happens in what
order*, which is exactly the part that has to reproduce on a replay.
"""
import pytest

from biguasim.server import actions as act
from biguasim.server.world import World, WorldError


def bare_world(tick=0, admin=()):
    """A World with no environment behind it.

    __init__ boots an engine; these tests only exercise the queue.
    """
    w = World.__new__(World)
    w._tick = tick
    w._input_delay = 3
    w._pending = []
    w._deferred = []
    w._controls = {}
    w._defaults = {}
    w._owner = {}
    w._external = set()
    w._admin = set(admin)
    w._record = None
    return w


ONE_OF_EACH = [
    act.SetControl(agent="uav0", command=[1.0, 2.0, 3.0, 4.0]),
    act.SetPose(agent="rig", position=(1.0, 2.0, 3.0)),
    act.SetControlDefaults(agent="uav0", command=[0.0] * 4),
    act.SpawnAgent(agent="uav1", agent_type="DjiMatrice", location=(1, 2, 3),
                   sensors=[{"sensor_type": "DynamicsSensor"}]),
    act.KillAgent(agent="uav1"),
    act.AddSensor(agent="uav0", sensor_type="RGBCamera", socket="CameraSocket"),
    act.RemoveSensor(agent="uav0", sensor_name="cam_1"),
    act.RotateSensor(agent="uav0", sensor_name="cam_1", rotation=(0, 45, 0)),
    act.SetWeather(weather="rain"),
    act.SetDayTime(hour=14),
    act.SetFogDensity(density=0.3),
]


@pytest.mark.parametrize("action", ONE_OF_EACH, ids=lambda a: a.kind)
def test_actions_survive_a_round_trip(action):
    assert act.decode(act.encode(action)) == action


def test_every_kind_is_covered_by_the_round_trip_test():
    assert {a.kind for a in ONE_OF_EACH} == set(act.kinds())


def test_unknown_kinds_are_rejected():
    """A client speaking a different protocol version should fail loudly."""
    with pytest.raises(ValueError, match="unknown action kind"):
        act.decode({"kind": "teleport_everyone"})


def test_unexpected_fields_are_rejected():
    payload = act.encode(act.SetControl(agent="uav0", command=[1.0]))
    payload["cheat"] = True
    with pytest.raises(ValueError, match="unexpected fields"):
        act.decode(payload)


def test_control_actions_are_not_world_mutating():
    """They are latest-wins and losing one is harmless, unlike a spawn."""
    assert not act.SetControl(agent="a", command=[]).mutates_world
    assert act.SpawnAgent(agent="a", agent_type="DjiMatrice").mutates_world


def test_due_order_ignores_arrival_order():
    """The heart of it: submission order must not reach the world."""
    world = bare_world(tick=5)
    scrambled = [
        act.SetControl(client_id="b", seq=1, target_tick=5, agent="a", command=[2]),
        act.SetControl(client_id="a", seq=2, target_tick=5, agent="a", command=[3]),
        act.SetControl(client_id="a", seq=1, target_tick=5, agent="a", command=[1]),
        act.SetControl(client_id="a", seq=1, target_tick=4, agent="a", command=[0]),
    ]
    world._pending = list(scrambled)
    forward = [a.order_key for a in world._due()]

    world = bare_world(tick=5)
    world._pending = list(reversed(scrambled))
    backward = [a.order_key for a in world._due()]

    assert forward == backward == sorted(forward)


def test_future_actions_are_held_back():
    world = bare_world(tick=5)
    world._pending = [
        act.SetControl(target_tick=5, agent="a", command=[1]),
        act.SetControl(target_tick=9, agent="a", command=[2]),
    ]
    assert [a.target_tick for a in world._due()] == [5]
    assert [a.target_tick for a in world._pending] == [9]


def test_deferred_actions_keep_their_place():
    """A slipped action stays behind whatever already sorted ahead of it."""
    world = bare_world(tick=7)
    world._deferred = [act.SpawnAgent(client_id="a", seq=1, target_tick=3,
                                      agent="x", agent_type="DjiMatrice")]
    world._pending = [act.SetControl(client_id="a", seq=1, target_tick=7,
                                     agent="a", command=[1])]
    assert [a.order_key for a in world._due()] == [(3, "a", 1), (7, "a", 1)]


def test_submit_moves_stale_actions_to_the_next_tick():
    """A tick that already ran cannot be changed, but the intent survives."""
    world = bare_world(tick=10)
    scheduled = world.submit(act.SetControl(target_tick=2, agent="a", command=[1]))
    assert scheduled == 11
    assert world._pending[0].target_tick == 11


def test_submit_leaves_future_actions_alone():
    world = bare_world(tick=10)
    assert world.submit(act.SetControl(target_tick=13, agent="a", command=[1])) == 13


def test_owners_are_enforced():
    world = bare_world()
    world._owner[("agent", "uav0")] = "alice"

    world.submit(act.SetControl(client_id="alice", agent="uav0", command=[1]))
    with pytest.raises(WorldError, match="may not act on agent"):
        world.submit(act.KillAgent(client_id="mallory", agent="uav0"))


def test_admins_bypass_ownership():
    world = bare_world(admin={"root"})
    world._owner[("agent", "uav0")] = "alice"
    world.submit(act.KillAgent(client_id="root", agent="uav0"))


def test_unowned_agents_are_open():
    """Agents that came from the scenario belong to nobody in particular."""
    world = bare_world()
    world.submit(act.SetControl(client_id="anyone", agent="uav0", command=[1]))


def test_next_tick_reflects_the_fixed_input_delay():
    """Fixed, not adaptive: adapting it makes a log replay differently."""
    world = bare_world(tick=100)
    assert world.next_tick == 103


def test_set_pose_is_not_world_mutating():
    """Like a control input: latest wins, and losing one is harmless."""
    assert not act.SetPose(agent="a", position=(1.0, 2.0, 3.0)).mutates_world


def test_set_pose_survives_a_round_trip():
    action = act.SetPose(agent="rig", position=(1.0, 2.0, 3.0),
                         rotation=(0.0, 45.0, 0.0), velocity=(0.0, 0.0, -1.0))
    assert act.decode(act.encode(action)) == action


def test_client_driven_agents_refuse_control_commands():
    """Two things deciding where a vehicle is, and no way to say which is right."""
    world = bare_world()
    world._controls["rig"] = None
    world._external.add("rig")

    with pytest.raises(WorldError, match="driven by its client"):
        world._set_control(act.SetControl(agent="rig", command=[1.0]))


def test_world_driven_agents_refuse_poses():
    world = bare_world()
    world._controls["uav0"] = None

    with pytest.raises(WorldError, match="driven by the world"):
        world._set_pose(act.SetPose(agent="uav0", position=(0.0, 0.0, 0.0)))
