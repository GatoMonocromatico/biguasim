"""Drawing between snapshots. No engine."""
import numpy as np
import pytest

from biguasim.client.interpolation import PoseBuffer, slerp

IDENTITY = [0.0, 0.0, 0.0, 1.0]


def pose(x, q=IDENTITY, v=(0.0, 0.0, 0.0)):
    return {"position": [x, 0.0, 0.0], "velocity": list(v), "quaternion": list(q)}


def buffer_with(count, step=0.05, delay=1.5):
    buf = PoseBuffer(delay=delay)
    for i in range(count):
        buf.push(i * step, {"a": pose(float(i))})
    return buf


def test_a_sample_between_snapshots_is_interpolated():
    buf = buffer_with(4)
    assert buf.sample(0.075)["a"]["position"][0] == pytest.approx(1.5)


def test_sampling_before_and_after_the_held_range_holds_still():
    """Extrapolating would invent motion. A frozen craft beats a wrong one."""
    buf = buffer_with(4)
    assert buf.sample(-10.0)["a"]["position"][0] == pytest.approx(0.0)
    assert buf.sample(10.0)["a"]["position"][0] == pytest.approx(3.0)


def test_the_render_time_lags_the_newest_snapshot():
    """The lag is what makes a late snapshot invisible instead of a stutter."""
    buf = buffer_with(5, step=0.05, delay=1.5)
    assert buf.interval == pytest.approx(0.05)
    assert buf.render_time() == pytest.approx(0.2 - 1.5 * 0.05)


def test_out_of_order_snapshots_are_dropped():
    """By the time a late one arrives its moment is drawn; using it jumps back."""
    buf = buffer_with(3)
    buf.push(0.025, {"a": pose(99.0)})
    assert len(buf) == 3
    assert buf.sample(0.05)["a"]["position"][0] == pytest.approx(1.0)


def test_history_is_bounded():
    buf = PoseBuffer(history=5)
    for i in range(50):
        buf.push(i * 0.05, {"a": pose(float(i))})
    assert len(buf) == 5


def test_agents_appear_as_soon_as_they_are_seen():
    buf = PoseBuffer(delay=0.0)
    buf.push(0.0, {"a": pose(0.0)})
    buf.push(0.05, {"a": pose(1.0), "b": pose(7.0)})
    assert "b" in buf.sample(0.025)


def test_agents_that_vanish_are_held_not_dragged():
    buf = PoseBuffer(delay=0.0)
    buf.push(0.0, {"a": pose(0.0), "b": pose(5.0)})
    buf.push(0.05, {"a": pose(1.0)})
    assert buf.sample(0.025)["b"]["position"][0] == pytest.approx(5.0)


def test_an_empty_buffer_samples_to_nothing():
    assert PoseBuffer().sample() == {}


def test_slerp_takes_the_shorter_way_round():
    """q and -q are the same orientation; the wrong sign spins the long way."""
    q0 = np.array([0.0, 0.0, 0.0, 1.0])
    q1 = -np.array([0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)])

    midpoint = slerp(q0, q1, 0.5)
    angle = 2 * np.degrees(np.arccos(np.clip(abs(float(np.dot(q0, midpoint))), -1, 1)))
    assert angle < 45.0, "should rotate the short way"


def test_slerp_endpoints_and_normalisation():
    q0 = np.array([0.0, 0.0, 0.0, 1.0])
    q1 = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])

    assert slerp(q0, q1, 0.0) == pytest.approx(q0)
    assert slerp(q0, q1, 1.0) == pytest.approx(q1)
    for fraction in (0.1, 0.5, 0.9):
        assert np.linalg.norm(slerp(q0, q1, fraction)) == pytest.approx(1.0)


def test_slerp_handles_nearly_identical_orientations():
    """The trig form divides by ~0 here, so it falls back to a straight lerp."""
    q0 = np.array([0.0, 0.0, 0.0, 1.0])
    q1 = np.array([1e-9, 0.0, 0.0, 1.0])
    assert np.all(np.isfinite(slerp(q0, q1, 0.5)))
