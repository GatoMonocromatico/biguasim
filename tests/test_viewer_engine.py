"""The viewer, drawing a world in a local engine.

Opt in with BIGUASIM_ENGINE_TESTS=1.

Driven by synthetic snapshots rather than a live world, so this needs one
engine rather than two. What it checks is the part that matters: that puppets
appear, follow what they are told, and go away again -- and that nothing here
simulates anything.
"""
import os
import sys

import numpy as np
import pytest

from biguasim.client import Viewer

pytestmark = [
    pytest.mark.engine,
    pytest.mark.skipif(os.environ.get("BIGUASIM_ENGINE_TESTS") != "1",
                       reason="needs a live engine; set BIGUASIM_ENGINE_TESTS=1"),
]

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

IDENTITY = [0.0, 0.0, 0.0, 1.0]


@pytest.fixture(scope="module")
def viewer():
    from determinism_probe import build
    cfg, _names, _ticks, _command = build("open")
    view = Viewer(cfg, show_viewport=False)
    try:
        yield view
    finally:
        view._env.__exit__(None, None, None)


def snapshot(tick, agents):
    return {"tick": tick, "time": tick * 0.05, "agents": agents}


def flying(x, y=0.0, z=30.0, kind="DjiMatrice"):
    return {"type": kind, "position": [x, y, z],
            "velocity": [1.0, 0.0, 0.0], "quaternion": IDENTITY}


def local_position(viewer, agent):
    state = viewer._env.tick()
    return np.asarray(state[agent][0]["DynamicsSensor"], dtype=np.float64)[6:9]


def test_a_viewer_starts_empty(viewer):
    """Nothing is assumed about the roster; puppets follow what arrives."""
    assert viewer.puppets == {}


def test_puppets_appear_for_agents_that_are_announced(viewer):
    for tick in range(6):
        viewer.feed(snapshot(tick, {"v0": flying(float(tick) * 2)}))
    for _ in range(6):
        viewer.draw()

    assert viewer.puppets == {"v0": "DjiMatrice"}
    assert viewer.tick == 5


def test_a_puppet_goes_where_it_is_told(viewer):
    for tick in range(6, 20):
        viewer.feed(snapshot(tick, {"v0": flying(float(tick) * 2)}))
    for _ in range(10):
        viewer.draw()

    target = viewer._buffer.sample()["v0"]["position"]
    # Teleports land on the following tick, so a little lag is expected; what
    # matters is that it is following rather than simulating its own flight.
    assert np.allclose(local_position(viewer, "v0"), target, atol=1.0)


def test_more_agents_appear_and_departed_ones_are_retired(viewer):
    for tick in range(20, 30):
        viewer.feed(snapshot(tick, {"v0": flying(float(tick) * 2),
                                    "v1": flying(0.0, y=float(tick))}))
    for _ in range(6):
        viewer.draw()
    assert sorted(viewer.puppets) == ["v0", "v1"]

    for tick in range(30, 40):
        viewer.feed(snapshot(tick, {"v0": flying(float(tick) * 2)}))
    for _ in range(8):
        viewer.draw()
    assert sorted(viewer.puppets) == ["v0"]


def test_the_viewer_never_simulates_anything(viewer):
    """Puppet mode: no dynamics locally, or the two would drift apart.

    The whole arrangement depends on the local copy deciding nothing.
    """
    assert viewer._env._dynamics_dict == {}

    frozen = viewer._buffer.sample()["v0"]["position"]
    for _ in range(20):
        viewer.draw()          # no new snapshots fed
    assert np.allclose(viewer._buffer.sample()["v0"]["position"], frozen), \
        "with no new state, a puppet must hold still rather than fly on"


def test_drawing_faster_than_snapshots_arrive_interpolates(viewer):
    """The reason for the delay: draw between snapshots, not on top of them."""
    seen = []
    for tick in range(40, 46):
        viewer.feed(snapshot(tick, {"v0": flying(float(tick) * 10)}))

    span = viewer._buffer.span
    for step in range(9):
        moment = span[0] + (span[1] - span[0]) * step / 8.0
        seen.append(float(viewer._buffer.sample(moment)["v0"]["position"][0]))

    assert seen == sorted(seen), "interpolation should advance monotonically"
    assert len(set(seen)) > 6, "should produce intermediate positions, not steps"
