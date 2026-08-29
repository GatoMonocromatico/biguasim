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
    """

    def __init__(self, address="127.0.0.1", port=8770, client_id=None,
                 scenario_cfg=None, timeout=5.0, stream_backlog=64):
        self._address = address
        self._port = port
        self._client_id = client_id or "client-{:x}".format(id(self) & 0xFFFFFF)
        self._build = proto.build_id(scenario_cfg or {})
        self._timeout = timeout
        self._seq = 0
        self._events = []
        self._info = None

        ctx = zmq.Context.instance()
        self._requests = ctx.socket(zmq.DEALER)
        self._requests.setsockopt(zmq.LINGER, 0)
        self._requests.connect("tcp://{}:{}".format(address, port))
        self._stream = ctx.socket(zmq.SUB)
        self._stream.setsockopt(zmq.LINGER, 0)
        # Bounded on purpose: a client slower than the world should lose old
        # messages rather than accumulate a backlog it reads as if it were now.
        self._stream.setsockopt(zmq.RCVHWM, int(stream_backlog))
        self._stream.connect("tcp://{}:{}".format(address, port + 1))

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

    def kill_agent(self, agent):
        """Retire an agent this client owns."""
        return self.submit(act.KillAgent(agent=agent))[0]

    def set_control(self, agent, command):
        """Drive an agent. Held until superseded."""
        return self.submit(act.SetControl(agent=agent, command=list(command)))[0]

    def set_control_defaults(self, agent, command):
        """What this agent should do if this client disappears."""
        return self.submit(act.SetControlDefaults(agent=agent,
                                                  command=list(command)))[0]

    def add_sensor(self, agent, sensor_type, **kwargs):
        """Attach a sensor. The world names it, so it cannot collide."""
        return self.submit(act.AddSensor(agent=agent, sensor_type=sensor_type,
                                         **kwargs))[0]

    def remove_sensor(self, agent, sensor_name):
        """Detach a sensor."""
        return self.submit(act.RemoveSensor(agent=agent,
                                            sensor_name=sensor_name))[0]

    def failures(self):
        """Take any 'that did not work' notices received so far.

        Returns:
            :obj:`list` of :obj:`dict`: Oldest first.
        """
        # Non-blocking sweep, in case notices are waiting but nothing has asked.
        while self._requests.poll(0):
            message = proto.unpack(self._requests.recv())
            if message.get("event"):
                self._events.append(message)
        events, self._events = self._events, []
        return events

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
