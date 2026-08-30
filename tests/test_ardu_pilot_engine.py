"""A pilot flying a live world, with a stand-in for ArduPilot.

Opt in with BIGUASIM_ENGINE_TESTS=1.

The stand-in speaks ArduPilot's real JSON FDM protocol -- the same 40-byte
servo packet, the same lockstep of send-then-block-for-state -- so everything
between the world and the flight controller is exercised for real: the world
service, ZeroMQ, tick assembly, the FDM handshake and the control path back.
What it does not exercise is ArduPilot's own behaviour, which is the point;
that is what flying it by hand is for.

The assertion that matters most here is the negative one: an agent must not
exist in the world before something is stabilising it.
"""
import json
import os
import socket
import struct
import threading
import time

import pytest

from biguasim.ardubridge import VEHICLE_REGISTRY
from biguasim.ardubridge.remote_runner import RemoteArduRunner
from biguasim.server.service import WorldService

pytestmark = [
    pytest.mark.engine,
    pytest.mark.skipif(os.environ.get("BIGUASIM_ENGINE_TESTS") != "1",
                       reason="needs a live engine; set BIGUASIM_ENGINE_TESTS=1"),
]

SERVO_FMT = "<HHI16H"
SERVO_MAGIC = 18458

PORT = 8793
PROFILE = VEHICLE_REGISTRY["DjiMatrice"]

SCENARIO = {
    "package_name": "SkyDive", "world": "Pier-Harbor", "main_agent": "uav0",
    "ticks_per_sec": 20, "frames_per_sec": 20,
    "octree_min": 0.02, "octree_max": 5.0,
    "agents": [{
        "agent_name": "uav0", "agent_type": "DjiMatrice",
        "sensors": [{"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
                     "configuration": {"UseCOM": True, "UseRPY": False}}],
        "dynamics": {"batch_size": 1},
        "control_abstraction": "cmd_motor_speeds",
        "location": [0, 0, 25], "rotation": [0, 0, -90],
    }],
}


class FakeSITL(threading.Thread):
    """ArduPilot's half of the JSON FDM protocol, and nothing else.

    Sends a servo packet, then blocks for the state that answers it, exactly as
    ``JSON::update()`` does. Resends when unanswered, which is how the real one
    copes with physics that has not started yet -- and, here, how it survives
    the pilot pausing to spawn the vehicle.
    """

    def __init__(self, port, pwm=1500):
        super().__init__(daemon=True)
        self.target = ("127.0.0.1", port)
        self.pwm = pwm
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.25)
        self.states = []
        self.sent = 0
        # Not _stop: threading.Thread already has one, and shadowing it breaks
        # join() in a way whose traceback points anywhere but here.
        self._halt = threading.Event()

    def _servo_packet(self, frame):
        return struct.pack(SERVO_FMT, SERVO_MAGIC, 400, frame,
                           *([self.pwm] * 16))

    def run(self):
        frame = 0
        while not self._halt.is_set():
            frame += 1
            self.sock.sendto(self._servo_packet(frame), self.target)
            self.sent += 1
            try:
                raw, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue                      # unanswered: resend, as AP does
            for line in raw.decode().splitlines():
                line = line.strip()
                if line:
                    self.states.append(json.loads(line))

    def stop(self):
        self._halt.set()
        self.join(timeout=3)
        self.sock.close()


@pytest.fixture(scope="module")
def service():
    svc = WorldService(SCENARIO, port=PORT, input_delay=2)
    thread = threading.Thread(target=svc.run, daemon=True)
    thread.start()
    time.sleep(1.0)
    try:
        yield svc
    finally:
        svc.stop()
        thread.join(timeout=10)
        svc.close()


_names = iter("pilot{}".format(n) for n in range(100))


@pytest.fixture
def pilot(service):
    # A fresh name per test, because a name is never reusable: kill_agent is a
    # soft kill and the engine keeps the actor. See _spawn_hint.
    runner = RemoteArduRunner(
        PROFILE, package_name="SkyDive", world="Pier-Harbor",
        agent=next(_names), port=PORT, instance=7,  # a port nothing else uses
        location=(6.0, 0.0, 25.0), verbose=False)
    try:
        yield runner
    finally:
        runner.close()


def test_the_vehicle_does_not_exist_until_its_pilot_does(service, pilot):
    """The whole reason start() blocks.

    A served world free-runs, so an agent spawned before its flight controller
    is integrated on whatever SetControl last said -- which for a quadrotor
    means falling. Spawning on connect removes that window entirely.
    """
    pilot.connect()

    started = threading.Event()
    error = []

    def start():
        try:
            pilot.start(timeout=30.0)
            started.set()
        except Exception as exc:                                  # noqa: BLE001
            error.append(exc)

    thread = threading.Thread(target=start, daemon=True)
    thread.start()

    # No SITL yet. The world must still not know about this vehicle.
    time.sleep(2.0)
    assert not started.is_set(), "start() returned without a flight controller"
    assert pilot.agent not in service._world.agents

    sitl = FakeSITL(pilot.fdm_port)
    sitl.start()
    try:
        assert started.wait(timeout=30), "start() never completed: {}".format(error)
        assert pilot.agent in service._world.agents
    finally:
        sitl.stop()
    thread.join(timeout=5)


def test_the_loop_feeds_ardupilot_and_steers_the_world(service, pilot):
    pilot.connect()
    sitl = FakeSITL(pilot.fdm_port)
    sitl.start()
    try:
        pilot.start(timeout=30.0)

        stop = threading.Event()
        thread = threading.Thread(
            target=lambda: pilot.run(should_stop=stop.is_set), daemon=True)
        thread.start()

        deadline = time.time() + 30
        while time.time() < deadline and len(sitl.states) < 15:
            time.sleep(0.1)
        stop.set()
        thread.join(timeout=5)

        assert len(sitl.states) >= 15, "the FDM loop never got going"

        # Every packet ArduPilot needs to run its EKF.
        first = sitl.states[0]
        for key in ("timestamp", "imu", "latitude", "longitude", "altitude",
                    "velocity", "quaternion"):
            assert key in first, "FDM packet missing {!r}".format(key)
        assert set(first["imu"]) == {"gyro", "accel_body"}

        # The clock must come from the world and must move forward: ArduPilot
        # derives its own timestep from consecutive values of this field.
        stamps = [s["timestamp"] for s in sitl.states]
        assert stamps == sorted(stamps)
        assert stamps[-1] > stamps[0]

        assert pilot.frames >= 15
        assert pilot.skipped <= pilot.frames // 4, \
            "skipped {} of {} ticks".format(pilot.skipped, pilot.frames)
        assert not pilot.failures()
    finally:
        sitl.stop()


def test_servo_output_becomes_motor_commands_in_the_world(service, pilot):
    """The control path back: PWM in, motor speeds applied to the agent."""
    pilot.connect()
    sitl = FakeSITL(pilot.fdm_port, pwm=1700)     # well above the 1100 cutoff
    sitl.start()
    try:
        pilot.start(timeout=30.0)

        stop = threading.Event()
        thread = threading.Thread(
            target=lambda: pilot.run(should_stop=stop.is_set), daemon=True)
        thread.start()

        deadline = time.time() + 30
        commanded = None
        while time.time() < deadline:
            commanded = service._world._controls.get(pilot.agent)
            if commanded and any(c > 0 for c in commanded):
                break
            time.sleep(0.1)
        stop.set()
        thread.join(timeout=5)

        assert commanded, "the world never received a control command"
        assert len(commanded) == PROFILE.num_motors
        assert all(c > 0 for c in commanded), \
            "PWM 1700 should be positive thrust, got {}".format(commanded)
    finally:
        sitl.stop()


def test_the_pilot_takes_the_tick_rate_from_the_world(service, pilot):
    """Not from a flag, a YAML file, or a default.

    The rate has to be right or the IMU cannot be sampled every tick, and a
    number the caller supplies is a number the caller can get wrong -- which
    surfaces much later as a refused spawn naming a rate nobody typed.
    """
    info = pilot.connect()

    assert info["ticks_per_sec"] == SCENARIO["ticks_per_sec"]
    assert pilot.ticks_per_sec == SCENARIO["ticks_per_sec"]

    imu = next(s for s in pilot.sensors if s["sensor_type"] == "IMUSensor")
    assert imu["Hz"] == SCENARIO["ticks_per_sec"], \
        "the IMU must be sampled every tick, whatever the world's rate is"


def test_a_sensor_rate_the_world_cannot_produce_is_refused_at_connect(service):
    """And refused with both numbers named, rather than at spawn with one."""
    runner = RemoteArduRunner(
        PROFILE, package_name="SkyDive", world="Pier-Harbor",
        agent=next(_names), port=PORT, instance=8,
        extra_sensors=[{"sensor_type": "RGBCamera", "Hz": 90}],
        verbose=False)
    try:
        with pytest.raises(RuntimeError, match="90 Hz.*ticks at 20 Hz"):
            runner.connect()
    finally:
        runner.close()
