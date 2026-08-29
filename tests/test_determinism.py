"""Determinism.

Whether the simulator is reproducible decides how replay has to work: if a run
can be re-executed from its inputs, an action log is enough and it is tiny. If
it cannot, replay has to ship recorded state instead, which is orders of
magnitude larger and cannot be modified and re-run.

The dynamics tests run anywhere. The engine tests need a world and are opt-in
via BIGUASIM_ENGINE_TESTS=1; they deliberately spawn separate processes,
because sharing an interpreter would not resemble replaying a log later.
"""
import os
import subprocess
import sys

import numpy as np
import pytest
import torch

from biguasim.dynamics.agents import ModelsFactory

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROBE = os.path.join(REPO, "tools", "determinism_probe.py")

needs_engine = pytest.mark.skipif(
    os.environ.get("BIGUASIM_ENGINE_TESTS") != "1",
    reason="needs a live engine; set BIGUASIM_ENGINE_TESTS=1 to run",
)


def dynamics_sequence(steps=300):
    """A fixed state/command stream, identical on every call."""
    rng = np.random.RandomState(1234)
    for t in range(steps):
        d = np.zeros(19, dtype=np.float32)
        d[0:3] = rng.uniform(-1, 1, 3)
        d[3:6] = rng.uniform(-5, 5, 3)
        d[6:9] = [0, 0, 40 + 0.01 * t]
        d[9:12] = rng.uniform(-0.5, 0.5, 3)
        d[12:15] = rng.uniform(-0.2, 0.2, 3)
        q = rng.uniform(-1, 1, 4)
        d[15:19] = q / np.linalg.norm(q)
        yield [{"DynamicsSensor": d}], [[300.0 + 10 * np.sin(t / 7.0)] * 4], 0.05 * (t + 1)


def run_dynamics(device):
    model = ModelsFactory.build_model("DjiMatrice")(
        batch_size=1, device=torch.device(device),
        control_abstraction="cmd_motor_speeds")
    out = []
    for state, control, t in dynamics_sequence():
        action = model.step(state, control, t)
        if torch.is_tensor(action):
            action = action.detach().cpu().numpy()
        out.append(np.asarray(action, dtype=np.float64).ravel())
    return np.stack(out)


@pytest.mark.parametrize("device", ["cpu",
    pytest.param("cuda", marks=pytest.mark.skipif(
        not torch.cuda.is_available(), reason="no CUDA"))])
def test_dynamics_are_reproducible(device):
    assert np.array_equal(run_dynamics(device), run_dynamics(device))


@pytest.mark.skipif(not torch.cuda.is_available(), reason="no CUDA")
def test_cpu_and_cuda_agree_to_float64_rounding():
    """They are not bit-identical, so a log records the device it ran on.

    environments.py picks cuda when it is available, and util.gpu() picks by
    free VRAM, so the device can change between runs on a multi-GPU host.
    """
    delta = np.abs(run_dynamics("cpu") - run_dynamics("cuda"))
    assert not np.array_equal(run_dynamics("cpu"), run_dynamics("cuda"))
    assert delta.max() < 1e-12


@needs_engine
@pytest.mark.engine
@pytest.mark.parametrize("scenario", ["open", "contact", "multi"])
def test_engine_is_reproducible_across_processes(scenario, tmp_path):
    """Separate interpreters, separate engine boots -- the shape replay takes.

    'contact' exercises collision resolution and raycasts, 'multi' exercises
    the engine's per-actor update ordering.
    """
    outs = []
    for run in ("a", "b"):
        out = tmp_path / "{}_{}.npz".format(scenario, run)
        subprocess.run(
            [sys.executable, PROBE, "--scenario", scenario, "--out", str(out)],
            check=True, cwd=REPO, timeout=600,
        )
        outs.append(np.load(str(out)))

    a, b = outs
    assert a.files, "probe recorded no channels"
    for key in a.files:
        assert np.array_equal(a[key], b[key]), "{} diverged in {}".format(key, scenario)
