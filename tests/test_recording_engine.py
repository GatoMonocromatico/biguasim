"""Recording and replaying a real world.

Opt in with BIGUASIM_ENGINE_TESTS=1.
"""
import os
import sys

import pytest

from biguasim.server import Recorder, Recording, World, replay
from biguasim.server import actions as act

pytestmark = [
    pytest.mark.engine,
    pytest.mark.skipif(os.environ.get("BIGUASIM_ENGINE_TESTS") != "1",
                       reason="needs a live engine; set BIGUASIM_ENGINE_TESTS=1"),
]

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

TICKS = 120


def scenario():
    from determinism_probe import build
    cfg, _names, _ticks, _command = build("open")
    return cfg


def a_script():
    """Two clients, a mid-run spawn, and control for both."""
    out = []
    for tick in range(10, TICKS - 20, 10):
        out.append(act.SetControl(client_id="bob", seq=tick, target_tick=tick,
                                  agent="uav0", command=[330.0 + tick % 7] * 4))
    out.append(act.SpawnAgent(
        client_id="alice", seq=1, target_tick=30, agent="uav5",
        agent_type="DjiMatrice", location=(8.0, 0.0, 25.0),
        sensors=[{"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
                  "configuration": {"UseCOM": True, "UseRPY": False}}]))
    for tick in range(40, TICKS - 20, 10):
        out.append(act.SetControl(client_id="alice", seq=500 + tick,
                                  target_tick=tick, agent="uav5",
                                  command=[320.0 + tick % 5] * 4))
    return out


def record(path, script=None):
    cfg = scenario()
    with Recorder(path, cfg, keyframe_every=20) as rec:
        with World(cfg, input_delay=3, record=rec.record_action) as world:
            for action in (script or a_script()):
                world.submit(action)
            for _ in range(TICKS):
                rec.observe(world.tick, world.step())
    return Recording(path)


def test_a_recording_replays_exactly(tmp_path):
    recording = record(str(tmp_path / "run.bslog"))
    assert recording.actions, "nothing was recorded"

    result = replay(recording)
    assert result["compared"] >= 4, "expected several keyframes to check"
    assert result["divergences"] == []


def test_a_recording_is_far_smaller_than_the_run(tmp_path):
    """Logging asks rather than state is what makes this worth doing."""
    path = tmp_path / "run.bslog"
    record(str(path))
    assert path.stat().st_size < 64 * 1024


def test_replay_re_executes_rather_than_playing_back(tmp_path):
    """Change one action and the outcome must change.

    If replay were playing recorded state back, editing an input would make no
    difference -- and the ability to ask "what if this had been different" is
    the whole reason to store asks instead of results.
    """
    recording = record(str(tmp_path / "run.bslog"))
    assert replay(recording)["divergences"] == []

    for index, (tick, action, error) in enumerate(recording.actions):
        if isinstance(action, act.SetControl):
            recording.actions[index] = (
                tick,
                act.SetControl(client_id=action.client_id, seq=action.seq,
                               target_tick=action.target_tick,
                               agent=action.agent, command=[200.0] * 4),
                error,
            )
            break

    assert replay(recording)["divergences"], "editing an action changed nothing"
