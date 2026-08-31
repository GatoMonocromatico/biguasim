"""A client's handle on a running world.

The client submits actions and reads what the world publishes. It never
advances anything: the world ticks whether or not this client is keeping up,
which is what makes a slow client harmless to everyone else.

Nothing here interpolates or predicts. Smoothing belongs in whatever draws the
scene, and keeping it out of this path is what guarantees that data used for
anything that matters is the state the world actually reported.
"""
import time

import numpy as np
import zmq

from biguasim.server import actions as act
from biguasim.server import protocol as proto


class RemoteError(Exception):
    """The world refused something, or did not answer."""


class RemoteWorld:
    """A connection to a :class:`~biguasim.server.service.WorldService`.

    Args:
        address (:obj:`str`, optional): Host of the world. Defaults to localhost.
        port (:obj:`int`, optional): Request port; state is on ``port + 1``.
        client_id (:obj:`str`, optional): Identifies this client to the world,
            and owns whatever it spawns. Defaults to a per-process id.
        scenario_cfg (:obj:`dict`, optional): The package and world this client
            has locally. Used for the build check at connect time.
        timeout (:obj:`float`, optional): Seconds to wait for a reply.
        stream_backlog (:obj:`int`, optional): How many published messages may
            queue before old ones are dropped. Small is right for watching --
            a viewer wants the present, not a backlog it will never catch up
            with. Raise it for a recorder, which wants every message.
        ipv6 (:obj:`bool`, optional): Allow IPv6. Defaults to True. ZeroMQ
            disables it per socket unless told otherwise, so without this an
            IPv6 address is accepted, connected to, and silently never reaches
            anything.

    An IPv6 address may be given plainly; the brackets ZeroMQ needs are added
    for you::

        RemoteWorld(address="2804:60:114:8b00::1", port=8770)
    """

    def __init__(self, address="127.0.0.1", port=8770, client_id=None,
                 scenario_cfg=None, timeout=5.0, stream_backlog=64, ipv6=True):
        self._address = address
        self._port = port
        self._client_id = client_id or "client-{:x}".format(id(self) & 0xFFFFFF)
        self._build = proto.build_id(scenario_cfg or {})
        self._timeout = timeout
        self._seq = 0
        self._events = []
        self._info = None
        # Submits sent without waiting for their ack. Replies carry no
        # correlation id, so _request would otherwise hand back one of
        # these instead of the answer it is waiting for.
        self._unacked = 0

        ctx = zmq.Context.instance()
        self._requests = ctx.socket(zmq.DEALER)
        self._stream = ctx.socket(zmq.SUB)
        for socket in (self._requests, self._stream):
            socket.setsockopt(zmq.LINGER, 0)
            # Set before connecting; ZeroMQ only reads it at socket creation.
            socket.setsockopt(zmq.IPV6, 1 if ipv6 else 0)
        # Bounded on purpose: a client slower than the world should lose old
        # messages rather than accumulate a backlog it reads as if it were now.
        self._stream.setsockopt(zmq.RCVHWM, int(stream_backlog))
        self._requests.connect(proto.endpoint(address, port))
        self._stream.connect(proto.endpoint(address, port + 1))

    # ------------------------------------------------------------- plumbing

    @property
    def client_id(self):
        """:obj:`str`: How the world knows this client."""
        return self._client_id

    @property
    def info(self):
        """:obj:`dict`: What the world said at connect time."""
        return self._info

    def _request(self, message):
        """Send a request and wait for its reply.

        Failure notices and collision corrections arrive unprompted on this same
        socket and can overtake a reply, so they are set aside as they appear
        rather than mistaken for one.

        Returns:
            :obj:`dict`: The reply.

        Raises:
            RemoteError: If nothing answered within the timeout.
        """
        message = dict(message, client_id=self._client_id)
        self._requests.send(proto.pack(message))

        deadline = time.time() + self._timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise RemoteError("no reply from the world at {}:{}".format(
                    self._address, self._port))
            if not self._requests.poll(int(remaining * 1000)):
                continue
            reply = proto.unpack(self._requests.recv())
            # Failure notices arrive on the same socket and may overtake a
            # reply, so they are set aside rather than mistaken for one.
            if reply.get("event"):
                self._events.append(reply)
                continue
            if self._unacked:
                # An ack for a stream_control() nobody is waiting on.
                self._unacked -= 1
                continue
            return reply

    def connect(self):
        """Introduce this client and check it is running the same world build.

        Returns:
            :obj:`dict`: The world's greeting -- current tick, roster, and the
            input delay actions should be aimed at.

        Raises:
            RemoteError: If the world refuses, usually a build mismatch.
        """
        reply = self._request({
            "op": proto.OP_HELLO,
            "protocol": proto.PROTOCOL_VERSION,
            "build": self._build,
        })
        if not reply.get("ok"):
            raise RemoteError(reply.get("error", "connection refused"))
        self._info = reply
        return reply

    def close(self):
        """Say goodbye, so the world can retire anything this client owned."""
        try:
            self._request({"op": proto.OP_BYE})
        except RemoteError:
            pass
        self._requests.close(linger=0)
        self._stream.close(linger=0)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # -------------------------------------------------------------- actions

    def submit(self, *actions):
        """Send actions to the world.

        Args:
            *actions (:class:`~biguasim.server.actions.Action`): What to do.

        Returns:
            :obj:`list` of :obj:`int`: The tick each was scheduled for. Not
            necessarily the tick asked for -- under load an action slips.

        Raises:
            RemoteError: If the world rejected any of them.
        """
        payloads = []
        for action in actions:
            self._seq += 1
            payloads.append(dict(act.encode(action), seq=self._seq,
                                 client_id=self._client_id))

        reply = self._request({"op": proto.OP_SUBMIT, "actions": payloads})
        if not reply.get("ok"):
            raise RemoteError(reply.get("error", "action rejected"))
        return reply.get("scheduled", [])

    def spawn_agent(self, agent, agent_type, **kwargs):
        """Add an agent. This client owns it until it disconnects."""
        return self.submit(act.SpawnAgent(agent=agent, agent_type=agent_type,
                                          **kwargs))[0]

    def spawn_ardupilot_agent(self, agent, agent_type, ardupilot=None, **kwargs):
        """Ask the world to fly a vehicle with a real ArduPilot.

        The world starts SITL and a bridge beside itself and answers with the
        ports, so nothing here needs to know which are free, where ArduPilot
        lives, or that its home has to match the world's GPS origin.

        The agent appears a moment later, created by the pilot once its flight
        controller is up rather than by this call -- an agent that exists before
        something is stabilising it does not wait, it falls.

        Args:
            agent (:obj:`str`): Name for the vehicle.
            agent_type (:obj:`str`): Vehicle type. Its registry entry decides
                which ArduPilot is run.
            ardupilot (:obj:`dict`, optional): Extra sim_vehicle options.
                ``params`` names a file in the world's parameter directory;
                any other key becomes a flag, so ``{"console": True,
                "speedup": 2}`` adds ``--console --speedup 2``.
            **kwargs: Passed to :class:`~biguasim.server.actions.SpawnAgent`,
                chiefly ``location`` and ``rotation``.

        Returns:
            :obj:`dict`: ``instance``, ``ports`` and the ``command`` that ran.
            ``ports["mavlink"]`` is what a GCS on this machine connects to;
            ``ports["gcs"]`` is where it listens for one anywhere else.

        Raises:
            RemoteError: If the world was not started with ``--allow-sitl``, or
                will not run what was asked.
        """
        self._seq += 1
        action = act.SpawnAgent(agent=agent, agent_type=agent_type,
                                ardupilot=dict(ardupilot or {}), **kwargs)
        payload = dict(act.encode(action), seq=self._seq,
                       client_id=self._client_id)
        reply = self._request({"op": proto.OP_SUBMIT, "actions": [payload]})
        if not reply.get("ok"):
            raise RemoteError(reply.get("error", "action rejected"))
        return reply.get("ardupilot", {}).get(agent, {})

    def kill_agent(self, agent):
        """Retire an agent this client owns."""
        return self.submit(act.KillAgent(agent=agent))[0]

    def set_control(self, agent, command):
        """Drive an agent. Held until superseded."""
        return self.submit(act.SetControl(agent=agent, command=list(command)))[0]

    def stream_control(self, agent, command):
        """Drive an agent without waiting for the world to acknowledge it.

        :meth:`set_control` costs a full round trip, which caps how often a
        client can steer at ``1/RTT``. That is fine for a script placing a
        vehicle and fatal for a flight controller closing a loop at the world's
        tick rate -- so this sends and moves on.

        Nothing is lost by not waiting. Control is latest-wins under
        zero-order hold, so a dropped command is superseded rather than
        missed, and a rejected one still surfaces through :meth:`failures`.

        Args:
            agent (:obj:`str`): Whose controls to set.
            command (sequence of :obj:`float`): The command.
        """
        self._seq += 1
        payload = dict(
            act.encode(act.SetControl(agent=agent, command=list(command))),
            seq=self._seq, client_id=self._client_id)
        self._requests.send(proto.pack({
            "op": proto.OP_SUBMIT,
            "actions": [payload],
            "client_id": self._client_id,
        }))
        self._unacked += 1
        self._drain_acks()

    def _drain_acks(self):
        """Clear replies owed to :meth:`stream_control`, without blocking.

        Left unread they would fill the socket's queue and, worse, be handed
        to the next :meth:`_request` as its answer.
        """
        while self._unacked and self._requests.poll(0):
            reply = proto.unpack(self._requests.recv())
            if reply.get("event"):
                self._events.append(reply)
            else:
                self._unacked -= 1

    def set_control_defaults(self, agent, command):
        """What this agent should do if this client disappears."""
        return self.submit(act.SetControlDefaults(agent=agent,
                                                  command=list(command)))[0]

    def set_pose(self, agent, position, rotation=(0.0, 0.0, 0.0),
                 velocity=(0.0, 0.0, 0.0), angular_velocity=(0.0, 0.0, 0.0)):
        """Place an agent this client integrates itself.

        Only for agents spawned with ``externally_driven=True``.
        """
        return self.submit(act.SetPose(
            agent=agent, position=tuple(position), rotation=tuple(rotation),
            velocity=tuple(velocity),
            angular_velocity=tuple(angular_velocity)))[0]

    def add_sensor(self, agent, sensor_type, **kwargs):
        """Attach a sensor. The world names it, so it cannot collide."""
        return self.submit(act.AddSensor(agent=agent, sensor_type=sensor_type,
                                         **kwargs))[0]

    def remove_sensor(self, agent, sensor_name):
        """Detach a sensor."""
        return self.submit(act.RemoveSensor(agent=agent,
                                            sensor_name=sensor_name))[0]

    def _pump_events(self):
        """Collect anything the world has sent unprompted."""
        while self._requests.poll(0):
            message = proto.unpack(self._requests.recv())
            if message.get("event"):
                self._events.append(message)
            elif self._unacked:
                # A stream_control() ack. Counted off here as well as in
                # _drain_acks, or _request would later skip a real reply.
                self._unacked -= 1

    def _take(self, kind):
        """Remove and return buffered events of one kind."""
        self._pump_events()
        taken = [e for e in self._events if e.get("event") == kind]
        self._events = [e for e in self._events if e.get("event") != kind]
        return taken

    def failures(self):
        """Take any 'that did not work' notices received so far.

        These arrive late by nature: whether an action works is not knowable
        until it runs, several ticks after it was accepted.

        Returns:
            :obj:`list` of :obj:`dict`: Oldest first.
        """
        return self._take("action_failed")

    def corrections(self):
        """Take collision corrections for agents this client drives itself.

        The world is authoritative on contact, so when a client-driven vehicle
        hits something the world says where it actually ended up. What to do
        about it is this client's decision -- snap to it, blend towards it, or
        ignore it and keep flying through the pier.

        Returns:
            :obj:`list` of :obj:`dict`: Each with ``tick``, ``agent``, ``pose``.
        """
        return self._take("correction")

    # ------------------------------------------------------------ streaming

    def watch_state(self):
        """Subscribe to the per-tick summary: who is where, and how fast.

        Kilobytes a tick regardless of how many clients watch, because the
        world publishes once and ZeroMQ fans it out.
        """
        self._stream.setsockopt(zmq.SUBSCRIBE, proto.TOPIC_STATE)

    def watch_sensor(self, agent, sensor):
        """Subscribe to one sensor's output.

        Unlike :meth:`watch_state`, this can be expensive -- an imaging sonar or
        a camera is megabytes a second -- which is exactly why it is opt-in and
        per sensor.
        """
        self._stream.setsockopt(zmq.SUBSCRIBE, proto.sensor_topic(agent, sensor))

    def unwatch_sensor(self, agent, sensor):
        """Stop receiving one sensor."""
        self._stream.setsockopt(zmq.UNSUBSCRIBE,
                                proto.sensor_topic(agent, sensor))

    def recv(self, timeout=None):
        """Wait for the next published message.

        Args:
            timeout (:obj:`float`, optional): Seconds. Blocks if omitted.

        Returns:
            :obj:`tuple` or None: ``(topic, message)``, or None on timeout.
            Sensor messages arrive with their payload already an array.
        """
        wait = -1 if timeout is None else int(timeout * 1000)
        if not self._stream.poll(wait):
            return None

        topic, raw = self._stream.recv_multipart()
        message = proto.unpack(raw)
        if "data" in message:
            message["data"] = np.frombuffer(
                message["data"], dtype=np.dtype(message["dtype"])
            ).reshape(message["shape"])
        return topic.decode(), message

    def states(self, count=None, timeout=None):
        """Iterate state messages as they arrive.

        Args:
            count (:obj:`int`, optional): Stop after this many.
            timeout (:obj:`float`, optional): Give up if nothing arrives.

        Yields:
            :obj:`dict`: Each state message.
        """
        seen = 0
        while count is None or seen < count:
            got = self.recv(timeout)
            if got is None:
                return
            topic, message = got
            if topic == proto.TOPIC_STATE.decode():
                seen += 1
                yield message

    def wait_for_tick(self, tick, timeout=10.0):
        """Read state until the world reaches a tick.

        Actions land in the future, so this is how a client waits for one it
        submitted to have actually happened.

        Args:
            tick (:obj:`int`): The tick to wait for.
            timeout (:obj:`float`, optional): Seconds to wait.

        Returns:
            :obj:`dict`: The first state at or after that tick.

        Raises:
            RemoteError: If the world did not get there in time.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            got = self.recv(max(0.05, deadline - time.time()))
            if got is None:
                continue
            topic, message = got
            if topic == proto.TOPIC_STATE.decode() and message["tick"] >= tick:
                return message
        raise RemoteError("world did not reach tick {} within {}s".format(
            tick, timeout))
