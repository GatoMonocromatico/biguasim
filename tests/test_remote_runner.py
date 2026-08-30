"""The pilot's tick assembly and port arithmetic. No engine, no sockets.

The runner's job between the world and ArduPilot is mostly bookkeeping: collect
one tick's sensors out of an interleaved stream, decide whether that tick is
usable, and hand it over in the right order. All of that is testable against a
stand-in stream, which is worth doing because the failure modes are quiet ones
-- a half-assembled tick becomes a malformed FDM packet, and a tick assembled
from two different ticks' sensors becomes a plausible-looking lie.
"""
import pytest

from biguasim.ardubridge import VEHICLE_REGISTRY
from biguasim.ardubridge.remote_runner import (
    FDM_BASE_PORT, MAVLINK_BASE_PORT, REQUIRED_SENSORS, RemoteArduRunner)

PROFILE = VEHICLE_REGISTRY["HolybroX500"]


class _Stream:
    """A world that replays a scripted list of published messages."""

    def __init__(self, messages):
        self.messages = list(messages)
        self.controls = []
        self.watched = []
        self.spawned = []
        self.defaults = []

    def recv(self, timeout=None):
        return self.messages.pop(0) if self.messages else None

    def stream_control(self, agent, command):
        self.controls.append((agent, list(command)))

    def connect(self):
        return {"tick": 0, "input_delay": 3, "agents": {}}

    def watch_state(self):
        self.watched.append("state")

    def watch_sensor(self, agent, sensor):
        self.watched.append("{}/{}".format(agent, sensor))

    def spawn_agent(self, agent, kind, **kwargs):
        self.spawned.append((agent, kind, kwargs))
        return 3

    def set_control_defaults(self, agent, command):
        self.defaults.append((agent, list(command)))

    def failures(self):
        return []

    def close(self):
        pass


class _Bridge:
    """An ArduPilot that always has one servo packet ready."""

    def __init__(self, pwm=None):
        self.sent = []
        self.pwm = pwm if pwm is not None else [1500] * 16
        self.frame = 0

    def bind(self):
        pass

    def build_json_state(self, frame, sim_time):
        return {"timestamp": sim_time, "sensors": sorted(frame)}

    def send_state(self, state):
        self.sent.append(state)

    def receive_pwm(self):
        self.frame += 1
        return self.frame, self.pwm

    def pwm_to_motor_cmds(self, pwm, frame):
        return [100.0, 200.0, 300.0, 400.0]

    def close(self):
        pass


def state(tick, time=None):
    return ("state", {"tick": tick, "time": time if time is not None else tick * 0.005,
                      "agents": {"uav0": {}}})


def sensor(tick, name, agent="uav0", data=None):
    return ("sensor/{}/{}".format(agent, name),
            {"tick": tick, "agent": agent, "sensor": name,
             "data": data if data is not None else [float(tick)]})


def full_tick(tick):
    """A state message plus every sensor ArduPilot needs."""
    return [state(tick)] + [sensor(tick, name) for name in REQUIRED_SENSORS]


def runner(messages, pwm=None):
    r = RemoteArduRunner.__new__(RemoteArduRunner)
    r._profile = PROFILE
    r._agent = "uav0"
    r._instance = 0
    r._verbose = False
    r._bridge = _Bridge(pwm)
    r.world = _Stream(messages)
    r._state_topic = "state"
    r._sensor_prefix = "sensor/uav0/"
    r._tick = None
    r._time = 0.0
    r._frame = {}
    r.frames = 0
    r.skipped = 0
    return r


def drain(r):
    """Pump until the scripted stream runs out, collecting serviced ticks."""
    serviced = []
    while r.world.messages:
        got = r.pump(timeout=0)
        if got is not None:
            serviced.append(got)
    return serviced


# --------------------------------------------------------------- assembly

def test_a_tick_is_serviced_once_the_next_one_starts():
    """The next state message is the marker that the previous tick is whole.

    No timer, no guessing at how many sensors were coming.
    """
    r = runner(full_tick(1) + full_tick(2))
    serviced = drain(r)

    assert len(serviced) == 1, "tick 2 is still open; only tick 1 is finished"
    frame, sim_time = serviced[0]
    assert sorted(frame) == sorted(REQUIRED_SENSORS)
    assert sim_time == pytest.approx(0.005)


def test_sensors_from_a_later_tick_do_not_contaminate_an_open_one():
    """A stray message tagged for another tick is dropped, not merged."""
    messages = [state(1), sensor(1, "IMUSensor"), sensor(7, "LocationSensor"),
                sensor(1, "LocationSensor"), sensor(1, "VelocitySensor"),
                sensor(1, "DynamicsSensor"), state(2)]
    r = runner(messages)
    (frame, _), = drain(r)

    assert frame["LocationSensor"] == [1.0], "took the tick-7 message"


def test_a_tick_missing_an_ekf_sensor_is_skipped_not_half_sent():
    """Half a frame is a malformed FDM packet, which is worse than none."""
    partial = [state(1)] + [sensor(1, n) for n in REQUIRED_SENSORS[:-1]]
    r = runner(partial + full_tick(2) + [state(3)])
    serviced = drain(r)

    assert len(serviced) == 1, "only the complete tick should be serviced"
    assert r.skipped == 1
    assert r.frames == 1


def test_another_agents_sensors_are_ignored():
    """Two vehicles share the stream; each pilot takes only its own."""
    messages = [state(1)] + [sensor(1, n) for n in REQUIRED_SENSORS] + \
               [sensor(1, "IMUSensor", agent="uav9", data=[99.0]), state(2)]
    r = runner(messages)
    (frame, _), = drain(r)

    assert frame["IMUSensor"] == [1.0], "took uav9's IMU"


# ---------------------------------------------------------------- ordering

def test_state_goes_out_before_servos_are_collected():
    """ArduPilot is blocked in recv_fdm; sending first is what unblocks it.

    Collecting servos first would read the previous frame's, pairing every
    command with the wrong state.
    """
    order = []
    r = runner(full_tick(1) + [state(2)])

    real_send = r._bridge.send_state
    real_recv = r._bridge.receive_pwm
    r._bridge.send_state = lambda s: (order.append("send"), real_send(s))[1]
    r._bridge.receive_pwm = lambda: (order.append("recv"), real_recv())[1]

    drain(r)
    assert order == ["send", "recv"]


def test_servos_reach_the_world_as_control():
    r = runner(full_tick(1) + [state(2)])
    drain(r)

    assert r.world.controls == [("uav0", [100.0, 200.0, 300.0, 400.0])]


def test_control_is_streamed_not_acknowledged():
    """set_control would put a round trip inside a loop running at tick rate."""
    r = runner(full_tick(1) + [state(2)])
    r.world.set_control = lambda *a: pytest.fail("must not wait for an ack")
    drain(r)

    assert r.world.controls, "nothing was sent at all"


def test_the_world_clock_is_used_verbatim():
    """ArduPilot derives its timestep from these, so they must be the world's.

    Counting ticks locally would drift the moment one is skipped.
    """
    r = runner(full_tick(4) + [state(5)])
    drain(r)

    assert r._bridge.sent[0]["timestamp"] == pytest.approx(0.02)


# ------------------------------------------------------------------ ports

@pytest.mark.parametrize("instance,fdm,mavlink", [
    (0, FDM_BASE_PORT, MAVLINK_BASE_PORT),
    (1, 9012, 5770),
    (3, 9032, 5790),
])
def test_instances_do_not_collide(instance, fdm, mavlink):
    """ArduPilot adds 10*instance to every port (SITL_cmdline.cpp:426)."""
    r = RemoteArduRunner.__new__(RemoteArduRunner)
    r._instance = instance
    assert r.fdm_port == fdm
    assert r.mavlink_port == mavlink


def test_the_default_sensor_list_is_exactly_what_the_ekf_needs():
    sensors = RemoteArduRunner.build_sensors(PROFILE, 200)
    assert sorted(s["sensor_type"] for s in sensors) == sorted(REQUIRED_SENSORS)


def test_extra_sensors_are_added_but_never_displace_the_ekf_ones():
    extra = [{"sensor_type": "RGBCamera", "Hz": 10},
             {"sensor_type": "IMUSensor", "Hz": 1}]      # a deliberate clash
    sensors = RemoteArduRunner.build_sensors(PROFILE, 200, extra=extra)
    types = [s["sensor_type"] for s in sensors]

    assert "RGBCamera" in types
    assert types.count("IMUSensor") == 1
    imu = next(s for s in sensors if s["sensor_type"] == "IMUSensor")
    assert imu["Hz"] == 200, "the EKF's IMU was replaced by the 1 Hz one"
