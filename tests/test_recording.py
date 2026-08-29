"""The recording format. No engine."""
import pytest

from biguasim.server import actions as act
from biguasim.server.recording import (
    FORMAT_VERSION, Recorder, Recording, current_device,
)

SCENARIO = {"package_name": "SkyDive", "world": "Pier-Harbor", "agents": []}


def write(path, records, **kwargs):
    with Recorder(str(path), SCENARIO, **kwargs) as rec:
        for tick, action, error in records:
            rec.record_action(tick, action, error)
    return Recording(str(path))


def test_actions_survive_the_file(tmp_path):
    written = [
        (3, act.SpawnAgent(client_id="a", seq=1, target_tick=3, agent="uav1",
                           agent_type="DjiMatrice", location=(1.0, 2.0, 3.0)), None),
        (5, act.SetControl(client_id="a", seq=2, target_tick=5, agent="uav1",
                           command=[1.0, 2.0, 3.0, 4.0]), None),
    ]
    recording = write(tmp_path / "r.bslog", written)

    assert [t for t, _, _ in recording.actions] == [3, 5]
    assert [a for _, a, _ in recording.actions] == [a for _, a, _ in written]


def test_failures_are_kept(tmp_path):
    """A faithful replay should fail the same way the original did."""
    recording = write(tmp_path / "r.bslog", [
        (7, act.SetControl(agent="ghost", command=[1.0]), "no such agent: 'ghost'"),
    ])
    assert recording.actions[0][2] == "no such agent: 'ghost'"


def test_the_scenario_is_stored_so_a_replay_can_rebuild_the_world(tmp_path):
    recording = write(tmp_path / "r.bslog", [])
    assert recording.scenario == SCENARIO


def test_the_device_is_recorded(tmp_path):
    """cpu and cuda agree only to rounding, so replay has to know which."""
    recording = write(tmp_path / "r.bslog", [])
    assert recording.device == current_device()
    assert recording.check_device(strict=True) is None


def test_replaying_on_a_different_device_is_refused(tmp_path):
    recording = write(tmp_path / "r.bslog", [], device="some-other-device")

    with pytest.raises(ValueError, match="agree only to float64 rounding"):
        recording.check_device(strict=True)

    warning = recording.check_device(strict=False)
    assert "some-other-device" in warning


def test_keyframes_land_on_their_interval(tmp_path):
    path = tmp_path / "r.bslog"
    with Recorder(str(path), SCENARIO, keyframe_every=10) as rec:
        for tick in range(35):
            rec.observe(tick, {"t": tick * 0.05,
                               "uav0": [{"DynamicsSensor": [float(tick)] * 19}]})
        assert rec.counts == (0, 4)          # ticks 0, 10, 20, 30

    recording = Recording(str(path))
    assert sorted(recording.keyframes) == [0, 10, 20, 30]
    assert recording.keyframes[20]["uav0"][0] == 20.0


def test_keyframes_can_be_switched_off(tmp_path):
    path = tmp_path / "r.bslog"
    with Recorder(str(path), SCENARIO, keyframe_every=0) as rec:
        for tick in range(20):
            rec.observe(tick, {"uav0": [{"DynamicsSensor": [1.0] * 19}]})
    assert Recording(str(path)).keyframes == {}


def test_an_empty_file_is_not_a_recording(tmp_path):
    path = tmp_path / "empty.bslog"
    path.write_bytes(b"")
    with pytest.raises(ValueError, match="not a recording"):
        Recording(str(path))


def test_the_action_log_is_small(tmp_path):
    """The reason to log asks rather than state: 200 ticks of it is nothing."""
    path = tmp_path / "r.bslog"
    with Recorder(str(path), SCENARIO, keyframe_every=0) as rec:
        for tick in range(200):
            rec.record_action(tick, act.SetControl(
                client_id="a", seq=tick, target_tick=tick,
                agent="uav0", command=[300.0] * 4))
    assert path.stat().st_size < 40 * 1024


def test_format_version_is_checked(tmp_path):
    recording = write(tmp_path / "r.bslog", [])
    assert recording.header["format"] == FORMAT_VERSION
