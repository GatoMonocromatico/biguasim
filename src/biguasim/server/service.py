"""Serving a :class:`~biguasim.server.world.World` over the network.

The service owns the tick loop. Clients never advance the world; they submit
actions between ticks and read what comes back, which is what lets several of
them share one simulation without coordinating with each other.

Every tick:

1. drain the request socket, admitting actions for a tick a little ahead,
2. advance the world,
3. publish state, and any sensor topic somebody subscribed to,
4. return failures to whoever caused them.

Pacing is the scenario's business, not this loop's. ``frames_per_sec`` in the
scenario throttles the engine to wall-clock time, and a world with people
watching almost always wants that: left free-running the engine will happily
tick hundreds of times a second, and every viewer is then permanently behind,
watching a backlog rather than the present. Batch runs want the opposite, which
is why the knob stays where it already is rather than being reinvented here.
"""
import time

import numpy as np
import zmq

from biguasim.server import protocol as proto
from biguasim.server.actions import decode
from biguasim.server.world import World, WorldError


class WorldService:
    """Runs a world and serves it.

    Args:
        scenario_cfg (:obj:`dict`): Starting scenario.
        port (:obj:`int`, optional): Request port. State is published on
            ``port + 1``. Defaults to 8770.
        bind (:obj:`str`, optional): Interface to bind. Defaults to all.
        admin_clients (:obj:`set`, optional): Clients exempt from ownership.
        record (callable, optional): Passed to :class:`~biguasim.server.world.World`.
        **world_kwargs: Passed to :class:`~biguasim.server.world.World`.
    """

    def __init__(self, scenario_cfg, port=8770, bind="*", admin_clients=None,
                 record=None, **world_kwargs):
        self._build = proto.build_id(scenario_cfg)
        self._world = World(scenario_cfg, admin_clients=admin_clients,
                            record=record, **world_kwargs)

        self._ctx = zmq.Context.instance()
        self._requests = self._ctx.socket(zmq.ROUTER)
        self._requests.bind("tcp://{}:{}".format(bind, port))
        self._publish = self._ctx.socket(zmq.PUB)
        self._publish.bind("tcp://{}:{}".format(bind, port + 1))

        self._clients = {}          # client id -> last seen time
        self._identities = {}       # client id -> ROUTER identity
        self._running = False

    @property
    def world(self):
        """:class:`~biguasim.server.world.World`: The world being served."""
        return self._world

    # -------------------------------------------------------------- requests

    def _drain_requests(self):
        """Handle everything waiting on the request socket."""
        while True:
            try:
                identity, raw = self._requests.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                return

            try:
                message = proto.unpack(raw)
                reply = self._handle(identity, message)
            except Exception as exc:                      # noqa: BLE001
                # A malformed message is one client's problem, not the world's.
                reply = {"ok": False, "error": "{}: {}".format(
                    type(exc).__name__, exc)}

            self._requests.send_multipart([identity, proto.pack(reply)])

    def _handle(self, identity, message):
        op = message.get("op")
        client_id = message.get("client_id", "")

        if op == proto.OP_HELLO:
            return self._hello(identity, message, client_id)

        if op == proto.OP_PING:
            self._clients[client_id] = time.time()
            return {"ok": True, "tick": self._world.tick}

        if op == proto.OP_BYE:
            killed = self._world.release_client(client_id)
            self._clients.pop(client_id, None)
            self._identities.pop(client_id, None)
            return {"ok": True, "killed": killed}

        if op == proto.OP_SUBMIT:
            self._clients[client_id] = time.time()
            self._identities[client_id] = identity
            return self._submit(message, client_id)

        return {"ok": False, "error": "unknown op: {!r}".format(op)}

    def _hello(self, identity, message, client_id):
        if message.get("protocol") != proto.PROTOCOL_VERSION:
            return {"ok": False, "error": "protocol {} != {}".format(
                message.get("protocol"), proto.PROTOCOL_VERSION)}

        # Refused rather than tolerated: a client rendering a different build
        # disagrees with the world about where things are, and nothing about
        # that failure looks like a version problem when you are staring at it.
        if message.get("build") != self._build:
            return {"ok": False, "error": "world build mismatch: client {!r}, "
                                          "server {!r}".format(message.get("build"), self._build)}

        self._clients[client_id] = time.time()
        self._identities[client_id] = identity
        return {
            "ok": True,
            "tick": self._world.tick,
            "next_tick": self._world.next_tick,
            "input_delay": self._world._input_delay,
            "agents": self._world.agents,
            "build": self._build,
        }

    def _submit(self, message, client_id):
        scheduled = []
        for payload in message.get("actions", []):
            payload = dict(payload)
            # The client's declared identity never overrides the connection's,
            # or one client could act as another simply by saying so.
            payload["client_id"] = client_id
            if not payload.get("target_tick"):
                payload["target_tick"] = self._world.next_tick
            try:
                scheduled.append(self._world.submit(decode(payload)))
            except (ValueError, WorldError) as exc:
                return {"ok": False, "error": str(exc), "scheduled": scheduled}
        return {"ok": True, "scheduled": scheduled, "tick": self._world.tick}

    # -------------------------------------------------------------- publish

    def _publish_tick(self, state):
        """Send the cheap summary every tick, plus any subscribed sensors.

        Subscription filtering happens in ZeroMQ, so the world sends sensor
        topics unconditionally and nothing crosses the network unless somebody
        asked. That keeps the decision out of the tick loop.
        """
        roster = {}
        for agent in self._world.agents:
            frames = state.get(agent)
            if not frames:
                continue
            dynamics = frames[0].get("DynamicsSensor")
            if dynamics is None:
                continue
            values = np.asarray(dynamics, dtype=np.float64)
            roster[agent] = {
                "position": values[6:9].tolist(),
                "velocity": values[3:6].tolist(),
                "quaternion": values[15:19].tolist(),
            }

        self._publish.send_multipart([proto.TOPIC_STATE, proto.pack({
            "tick": self._world.tick,
            "time": float(state.get("t", 0.0)),
            "agents": roster,
        })])

        for agent in self._world.agents:
            frames = state.get(agent)
            if not frames:
                continue
            for sensor, value in frames[0].items():
                if value is None:
                    continue
                array = np.ascontiguousarray(value)
                self._publish.send_multipart([
                    proto.sensor_topic(agent, sensor),
                    proto.pack({
                        "tick": self._world.tick,
                        "agent": agent,
                        "sensor": sensor,
                        "dtype": array.dtype.str,
                        "shape": list(array.shape),
                        "data": array.tobytes(),
                    }),
                ])

    def _report_failures(self):
        """Tell clients about actions of theirs that did not work."""
        for tick, action, message in self._world.drain_errors():
            identity = self._identities.get(action.client_id)
            if identity is None:
                continue
            self._requests.send_multipart([identity, proto.pack({
                "ok": False, "event": "action_failed", "tick": tick,
                "kind": action.kind, "error": message,
            })])

    # ------------------------------------------------------------------ run

    def step(self):
        """One full cycle: requests in, world forward, state out."""
        self._drain_requests()
        state = self._world.step()
        self._publish_tick(state)
        self._report_failures()
        return state

    def run(self, ticks=None):
        """Serve until stopped.

        Args:
            ticks (:obj:`int`, optional): Stop after this many. Runs forever if
                omitted, which is the normal case.
        """
        self._running = True
        served = 0
        try:
            while self._running and (ticks is None or served < ticks):
                self.step()
                served += 1
        finally:
            self._running = False

    def stop(self):
        """Ask :meth:`run` to return after the current tick."""
        self._running = False

    def close(self):
        """Shut down sockets and the world."""
        self._requests.close(linger=0)
        self._publish.close(linger=0)
        self._world.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
