"""ArduPilot SITL against a BiguaSim world running in another process.

The counterpart to :class:`~biguasim.ardubridge.runner.ArduBiguaSimRunner`. That
one owns a ``biguasim.make()`` environment, which means it owns the UE5 binary,
which means exactly one of them can exist. This one owns only a connection, so
several vehicles -- from several people -- can fly in one simulation and see
each other in it.

Where to run this
=================

**On the machine running the world.** Not as a preference: ArduPilot's JSON
backend is a lockstep protocol. ``JSON::update()`` is ``output_servos()``
followed by a blocking ``recv_fdm()``, one frame in flight, and the servos for
frame N+1 are computed from the state at frame N. That is a control dependency,
so nothing about it can be pipelined, and a round trip across a network would
cap the loop at ``1/RTT``.

Co-located, the loop runs over loopback at about 0.05 ms and the question
disappears. What crosses the network instead is MAVLink, which was designed for
telemetry radios and does not care about 45 ms.

It is the same split the real airframe already uses: the flight controller sits
next to the vehicle, and the ground station is far away over a slow link.

The vehicle exists only while its pilot does
============================================

:meth:`RemoteArduRunner.start` binds the FDM socket, waits for SITL's opening
servo packet, and *only then* spawns the agent.

That ordering matters more than it looks. The local bridge cannot simulate
without ArduPilot -- ``env.step()`` is only called when a servo frame arrives --
but a served world free-runs on its own loop. An agent spawned before its
flight controller is up does not wait politely; it is integrated with whatever
``SetControl`` last said, and a quadrotor with no controller falls out of the
sky. Spawning on connect means that moment never exists.

ArduPilot is blocked in ``recv_fdm`` while the spawn happens, which costs
nothing: ``time_now_us`` only advances on a received frame, so the wait is
invisible to the flight controller's own clock.
"""

from __future__ import annotations

import time

from .bridge import ArduPilotBridge
from .runner import ArduBiguaSimRunner
from .vehicle import VehicleProfile

_DEFAULT_GPS_ORIGIN = (33.810313, -118.393867)

#: ArduPilot's EKF is fed from these four. A tick missing any of them cannot be
#: turned into an FDM packet, so it is skipped rather than half-sent.
REQUIRED_SENSORS = ("IMUSensor", "LocationSensor",
                    "VelocitySensor", "DynamicsSensor")

#: ArduPilot adds ``10 * instance`` to every port it uses
#: (``SITL_cmdline.cpp:426-441``), so one offset per vehicle is the whole of
#: multi-vehicle port allocation.
PORTS_PER_INSTANCE = 10

#: Where SITL sends its servo output, and so where the bridge listens
#: (``SIM_OUT_PORT`` in ``SITL_cmdline.cpp``).
FDM_BASE_PORT = 9002

#: SITL's MAVLink TCP port (``BASE_PORT``). Not used here -- the pilot never
#: speaks MAVLink -- but reported so a caller knows where to point a GCS.
MAVLINK_BASE_PORT = 5760

#: Below this the pilot warns. A multirotor attitude loop is closed at a few
#: hundred hertz, and the world's tick rate is what it actually gets; at
#: serve_world's default of 20 the vehicle will not arm, let alone fly.
ADVISED_FLIGHT_RATE = 200


def _spawn_hint(agent, error):
    """Explain a duplicate-name refusal, which is not what it looks like.

    Killing an agent is a *soft* kill: the world stops simulating it and drops
    it from the roster, but the engine keeps the actor, because nothing in the
    plugin destroys one. So the name is never released, and a pilot restarted
    under its old name is refused -- by the engine, with a message that says
    nothing about why.

    Until a DespawnAgent command exists in the UE5 plugin, a restarted pilot
    needs a new agent name.
    """
    text = str(error)
    if "uplicate" in text or "already exists" in text:
        return ("the world already has an agent called {!r}. Killing an agent "
                "is a soft kill -- the engine keeps the actor and never "
                "releases the name -- so a restarted pilot needs a new "
                "--agent. Original error: {}".format(agent, text))
    return "the world refused the spawn: {}".format(text)


class RemoteArduRunner:
    """Bridges one ArduPilot SITL to one agent in a world running elsewhere.

    Args:
        profile (:class:`~biguasim.ardubridge.vehicle.VehicleProfile`): The
            vehicle, from ``VEHICLE_REGISTRY``.
        package_name (:obj:`str`): World package. Must match the world's, or the
            connection is refused -- mismatched collision geometry looks like
            broken physics rather than a version problem.
        world (:obj:`str`): World name, same rule.
        agent (:obj:`str`, optional): Agent name. Defaults to the profile's.
        address (:obj:`str`, optional): Where the world is. Loopback by default,
            which is the only value that should be used in anger.
        port (:obj:`int`, optional): The world's request port. State is on
            ``port + 1``.
        instance (:obj:`int`, optional): ArduPilot SITL instance number. Shifts
            every port by ``10 * instance``, which is how a second vehicle
            avoids the first.
        extra_sensors (:obj:`list`, optional): Sensor specs to add on top of
            the ones ArduPilot needs -- cameras, a rangefinder, anything a ROS
            stack wants. The ArduPilot set is built at connect time from the
            rate the world reports, so it is never out of step with it.
        location, rotation (sequence of :obj:`float`, optional): Where to spawn.
        dynamics (:obj:`dict`, optional): Overrides for the dynamics model.
        gps_origin (:obj:`tuple`, optional): Latitude and longitude the world
            origin maps to.
        client_id (:obj:`str`, optional): How the world should know this pilot.
        stream_backlog (:obj:`int`, optional): Bounded receive queue. A pilot
            that falls behind drops old ticks rather than reading a backlog as
            though it were the present.
    """

    def __init__(self, profile, *, package_name, world, agent=None,
                 address="127.0.0.1", port=8770, instance=0, extra_sensors=None,
                 location=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0),
                 dynamics=None,
                 gps_origin=_DEFAULT_GPS_ORIGIN, client_id=None,
                 stream_backlog=256, verbose=True):
        # Imported here rather than at module scope so that importing
        # biguasim.ardubridge on a machine without pyzmq still works -- the
        # local runner has no use for it.
        from biguasim.client.remote import RemoteWorld

        self._profile = profile
        self._agent = agent or (profile.name.lower() + "0")
        self._instance = int(instance)
        self._location = tuple(location)
        self._rotation = tuple(rotation)
        self._dynamics = dict(dynamics or {})
        self._verbose = verbose
        self._started = False

        self._extra_sensors = [dict(spec) for spec in (extra_sensors or [])]
        #: Filled in by :meth:`connect`, once the world has said how fast it
        #: ticks. Building it earlier would mean guessing that number.
        self.sensors = None
        #: The world's tick rate, as reported on connect.
        self.ticks_per_sec = None

        self._bridge = ArduPilotBridge(
            profile, port=self.fdm_port, gps_origin=gps_origin)

        self.world = RemoteWorld(
            address=address, port=port, stream_backlog=stream_backlog,
            client_id=client_id or "pilot-{}".format(self._agent),
            scenario_cfg={"package_name": package_name, "world": world},
        )

        self._state_topic = "state"
        self._sensor_prefix = "sensor/{}/".format(self._agent)

        # Assembly state for the tick currently arriving.
        self._tick = None
        self._time = 0.0
        self._frame = {}
        self.frames = 0
        self.skipped = 0

    # ------------------------------------------------------------- geometry

    @property
    def agent(self):
        """:obj:`str`: The agent this pilot flies."""
        return self._agent

    @property
    def fdm_port(self):
        """:obj:`int`: Where this pilot listens for SITL's servo output."""
        return FDM_BASE_PORT + PORTS_PER_INSTANCE * self._instance

    @property
    def mavlink_port(self):
        """:obj:`int`: Where this pilot's SITL serves MAVLink over TCP."""
        return MAVLINK_BASE_PORT + PORTS_PER_INSTANCE * self._instance

    @staticmethod
    def build_sensors(profile, ticks_per_sec=200, extra=None):
        """The sensors ArduPilot needs, plus anything else asked for.

        Reuses :meth:`ArduBiguaSimRunner.build_scenario` rather than restating
        the list, so the EKF's inputs cannot drift between the local and remote
        paths. A sensor named in both is taken from ``build_scenario``: getting
        the EKF's inputs wrong is a far worse failure than a camera at an
        unexpected rate.

        Args:
            profile (:class:`VehicleProfile`): The vehicle.
            ticks_per_sec (:obj:`int`, optional): Used to rate the IMU.
            extra (:obj:`list`, optional): Additional sensor specs.

        Returns:
            :obj:`list` of :obj:`dict`: Sensor specs, ready for ``spawn_agent``.
        """
        scenario = ArduBiguaSimRunner.build_scenario(
            profile, package_name="", world="", ticks_per_sec=ticks_per_sec)
        sensors = list(scenario["agents"][0]["sensors"])
        have = {spec["sensor_type"] for spec in sensors}
        for spec in extra or []:
            if spec["sensor_type"] not in have:
                sensors.append(dict(spec))
        return sensors

    # -------------------------------------------------------------- startup

    def connect(self):
        """Introduce the pilot, learn the world's tick rate, size the sensors.

        The rate is asked for rather than configured. ArduPilot's IMU has to be
        sampled every tick, and a sensor rate has to divide the tick rate
        exactly, so a pilot carrying its own idea of that number is a pilot
        that can carry the wrong one -- which surfaces much later as a refused
        spawn that names a rate nobody typed.

        Returns:
            :obj:`dict`: The world's greeting.

        Raises:
            RuntimeError: If the world does not report its tick rate, or an
                extra sensor asks for a rate this world cannot produce.
        """
        info = self.world.connect()

        rate = info.get("ticks_per_sec")
        if not rate:
            raise RuntimeError(
                "this world did not report its tick rate, so it is running "
                "older code than this pilot. Restart it from the same "
                "checkout.")
        self.ticks_per_sec = int(rate)

        self._check_rates()
        self.sensors = self.build_sensors(
            self._profile, self.ticks_per_sec, extra=self._extra_sensors)

        self.world.watch_state()
        for spec in self.sensors:
            self.world.watch_sensor(
                self._agent, spec.get("sensor_name", spec["sensor_type"]))

        self._say("connected at tick {}, world ticks at {} Hz".format(
            info.get("tick"), self.ticks_per_sec))
        if self.ticks_per_sec < ADVISED_FLIGHT_RATE:
            self._say(
                "WARNING: {} Hz is well below the {} Hz an ArduPilot attitude "
                "loop wants. Expect it to arm badly or not at all -- start the "
                "world with --rate {}.".format(
                    self.ticks_per_sec, ADVISED_FLIGHT_RATE, ADVISED_FLIGHT_RATE))
        return info

    def _check_rates(self):
        """Reject an extra sensor this world cannot sample, and say why.

        The world refuses these too, but not until the spawn runs, and its
        message names a rate the caller never typed -- it came from a default.
        Caught here it can name both numbers and where they came from.
        """
        for spec in self._extra_sensors:
            hz = spec.get("Hz")
            if hz is None:
                continue
            name = spec.get("sensor_name", spec["sensor_type"])
            if hz > self.ticks_per_sec or self.ticks_per_sec % hz:
                raise RuntimeError(
                    "sensor {!r} asks for {} Hz, but this world ticks at {} Hz. "
                    "A sensor rate is a tick divider, so it must divide that "
                    "exactly and cannot exceed it.".format(
                        name, hz, self.ticks_per_sec))

    def start(self, timeout=120.0):
        """Wait for SITL, then spawn the agent it will fly.

        Blocks until ArduPilot sends its first servo packet, so nothing exists
        in the world until there is something to stabilise it. Its values are
        discarded: at boot ArduPilot is disarmed and every channel converts to
        zero thrust anyway, and it resends servos once a second until answered.

        Args:
            timeout (:obj:`float`, optional): How long to wait for SITL, and
                then for the agent to appear.

        Raises:
            RuntimeError: If SITL never appears, the world refuses the spawn, or
                the agent never shows up in the roster.
        """
        if self.sensors is None:
            raise RuntimeError("call connect() before start()")
        self._bridge.bind()
        self._say("listening for SITL on udp/{} -- start it now"
                  .format(self.fdm_port))

        deadline = time.time() + timeout
        while time.time() < deadline:
            frame, _ = self._bridge.receive_pwm()
            if frame is not None:
                break
        else:
            raise RuntimeError(
                "no servo packet on udp/{} within {}s -- is SITL running with "
                "--model JSON and this instance number?".format(
                    self.fdm_port, timeout))

        self._say("SITL is up; spawning {}".format(self._agent))
        try:
            self.world.spawn_agent(
                self._agent, self._profile.name,
                location=self._location, rotation=self._rotation,
                control_abstraction=self._profile.control_abstraction,
                sensors=self.sensors, dynamics=self._dynamics)
        except Exception as exc:                                   # noqa: BLE001
            raise RuntimeError(_spawn_hint(self._agent, exc)) from exc
        # Set the moment the spawn is accepted, not once it is confirmed: if
        # the confirmation below times out the agent may still turn up, and
        # close() has to be willing to retire it. A name is never reusable, so
        # an agent leaked here is one nobody can replace.
        self._started = True

        # A pilot that dies mid-flight must not leave the world holding the
        # last throttle it was given.
        self.world.set_control_defaults(
            self._agent, [0.0] * self._profile.num_motors)

        self._await_agent(deadline)

    def _await_agent(self, deadline):
        """Confirm the spawn actually happened.

        Whether a spawn works is not knowable when it is accepted -- it runs in
        the world several ticks later -- so refusals arrive asynchronously.
        Without this check, a vehicle type the world cannot build is
        indistinguishable from a healthy pilot that never receives anything.
        """
        while time.time() < deadline:
            for failure in self.world.failures():
                raise RuntimeError(
                    _spawn_hint(self._agent, failure.get("error")))
            got = self.world.recv(timeout=0.5)
            if got is None:
                continue
            topic, message = got
            if topic == self._state_topic and self._agent in message.get("agents", {}):
                self._say("{} is in the world at tick {}".format(
                    self._agent, message["tick"]))
                return
        raise RuntimeError(
            "{} never appeared in the world".format(self._agent))

    # ----------------------------------------------------------------- loop

    def pump(self, timeout=1.0):
        """Read one message, and service a tick if that completed one.

        The stream carries a state message per tick followed by that tick's
        sensor messages, in order, so the *next* state message is the marker
        that the previous tick is complete. That is why a frame is emitted on
        the boundary rather than after a timer -- no guessing, and no waiting
        for sensors that were never coming.

        Args:
            timeout (:obj:`float`, optional): Seconds to wait for a message.

        Returns:
            :obj:`tuple` or None: ``(frame, sim_time)`` if a tick was serviced,
            where ``frame`` maps sensor name to its data. None otherwise.
        """
        got = self.world.recv(timeout=timeout)
        if got is None:
            return None
        topic, message = got

        if topic == self._state_topic:
            serviced = None
            if self._tick is not None:
                if all(name in self._frame for name in REQUIRED_SENSORS):
                    serviced = (self._frame, self._time)
                    self._service(self._frame, self._time)
                    self.frames += 1
                else:
                    # A tick lost to the receive high-water mark. Reported to
                    # nobody: ArduPilot derives its timestep from the
                    # timestamps we send, so it simply sees a larger one.
                    self.skipped += 1
            self._tick = message["tick"]
            self._time = message["time"]
            self._frame = {}
            return serviced

        if topic.startswith(self._sensor_prefix) and message.get("tick") == self._tick:
            self._frame[message["sensor"]] = message["data"]
        return None

    def _service(self, frame, sim_time):
        """One tick: tell SITL where it is, then take its servos.

        Order matters. ArduPilot is blocked in ``recv_fdm`` waiting for state,
        so sending it first is what lets SITL run at all; only then is there a
        servo packet to collect.

        Control goes out with ``stream_control`` rather than ``set_control``.
        The latter waits for an acknowledgement, which would put a round trip
        back inside a loop that runs at the world's tick rate. Nothing is lost:
        control is latest-wins under zero-order hold, so a dropped command is
        superseded rather than missed, and a rejected one still surfaces
        through :meth:`failures`.
        """
        json_state = self._bridge.build_json_state(frame, sim_time)
        if json_state:
            self._bridge.send_state(json_state)

        pwm_frame, pwm = self._bridge.receive_pwm()
        if pwm_frame is not None:
            self.world.stream_control(
                self._agent, self._bridge.pwm_to_motor_cmds(pwm, pwm_frame))

    def run(self, on_frame=None, should_stop=None, report_every=0):
        """Fly until interrupted.

        Args:
            on_frame (callable, optional): Called ``(frame, sim_time)`` after
                each serviced tick. This is where a ROS bridge publishes.
            should_stop (callable, optional): Polled each pass; truthy ends the
                loop.
            report_every (:obj:`int`, optional): Print a frame/skip count every
                N ticks. 0 is silent.
        """
        try:
            while not (should_stop and should_stop()):
                serviced = self.pump()
                if serviced is None:
                    continue
                if on_frame is not None:
                    on_frame(*serviced)
                total = self.frames + self.skipped
                if report_every and total % report_every == 0:
                    self._say("tick {}  frames {}  skipped {}".format(
                        self._tick, self.frames, self.skipped))
        except KeyboardInterrupt:
            pass

    # ---------------------------------------------------------------- close

    def failures(self):
        """Anything the world rejected since this was last asked."""
        return self.world.failures()

    def close(self):
        """Retire the vehicle and hang up.

        Killing the agent is the symmetric half of spawning it on connect: the
        vehicle exists exactly as long as its flight controller does. Without
        it a pilot cannot be restarted -- the world still holds the agent, and
        the next spawn is refused as a duplicate.

        The zero-throttle defaults registered in :meth:`start` cover the other
        case, where this never runs at all. Then the vehicle outlives its pilot
        on a safe standing order rather than on whatever throttle it was last
        given.
        """
        if self._started:
            try:
                self.world.kill_agent(self._agent)
            except Exception as exc:                               # noqa: BLE001
                self._say("could not retire the agent: {}".format(exc))
            self._started = False
        self._bridge.close()
        try:
            self.world.close()
        except Exception:                                          # noqa: BLE001
            pass

    def _say(self, message):
        if self._verbose:
            print("[pilot {}] {}".format(self._agent, message))

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()
        return False
