"""Sensor lifecycle against a live UE5 world.

Opt in with BIGUASIM_ENGINE_TESTS=1; needs an installed SkyDive package and a
GPU, and takes about a minute to boot the engine.

    BIGUASIM_ENGINE_TESTS=1 pytest tests/test_sensor_lifecycle_engine.py
"""
import glob
import os

import numpy as np
import pytest

import biguasim
from biguasim.sensors import SensorDefinition

pytestmark = pytest.mark.skipif(
    os.environ.get("BIGUASIM_ENGINE_TESTS") != "1",
    reason="needs a live engine; set BIGUASIM_ENGINE_TESTS=1 to run",
)

CYCLES = 15

SCENARIO = {
    "package_name": "SkyDive",
    "world": "Pier-Harbor",
    "main_agent": "uav0",
    "ticks_per_sec": 20,
    "frames_per_sec": False,
    "octree_min": 0.02,
    "octree_max": 5.0,
    "agents": [
        {
            "agent_name": "uav0",
            "agent_type": "DjiMatrice",
            "sensors": [
                {
                    "sensor_type": "DynamicsSensor",
                    "socket": "IMUSocket",
                    "configuration": {"UseCOM": True, "UseRPY": False},
                },
                {"sensor_type": "LocationSensor", "socket": "IMUSocket"},
            ],
            "dynamics": {"batch_size": 1},
            "control_abstraction": "cmd_motor_speeds",
            "location": [0, 0, 40],
            "rotation": [0, 0, -90],
        }
    ],
}


@pytest.fixture(scope="module")
def env():
    with biguasim.make(scenario_cfg=SCENARIO, show_viewport=False) as environment:
        environment.reset()
        for _ in range(5):
            environment.tick()
        yield environment


def camera(index):
    return SensorDefinition(
        "uav0-id0", "DjiMatrice", "churn{}".format(index),
        "RGBCamera", socket="CameraSocket",
    )


def test_sensor_churn_does_not_leak(env):
    agent = env.agents["uav0-id0"]

    agent.add_sensors(camera(0))
    env.tick()
    agent.remove_sensors(camera(0))
    env.tick()

    blocks = len(glob.glob("/dev/shm/HOLODECK_MEM*"))
    fds = len(os.listdir("/proc/{}/fd".format(os.getpid())))

    for i in range(CYCLES):
        agent.add_sensors(camera(i))
        env.tick()
        agent.remove_sensors(camera(i))
        env.tick()

    assert len(glob.glob("/dev/shm/HOLODECK_MEM*")) == blocks
    assert len(os.listdir("/proc/{}/fd".format(os.getpid()))) == fds


def test_sensors_still_live_after_reset(env):
    """Fails if clear() ever unlinks: Python would map a new inode while the
    engine keeps writing to the old one, and this reads zeros forever."""
    env.reset()
    for _ in range(10):
        state = env.tick()

    assert np.any(state["uav0"][0]["LocationSensor"] != 0)
