"""Watching a world from your own machine, in your own engine.

The viewer runs a full local copy of the world and puts it in puppet mode: no
dynamics, no sensor simulation, nothing decided locally. Every tick it moves
each vehicle to where the world says it is and draws the result.

This is why watching costs the world almost nothing. The wire carries poses --
a few kilobytes a tick, published once however many people are looking -- and
the pixels are made on the machine that wants them, in the same engine, with
the same assets, at full fidelity. Ten people watching cost the world the same
as one.

The camera is not part of any of this. Where a viewer looks is nobody else's
business and never leaves the machine. A viewer that wants something the world
must actually compute -- a photoreal camera feed, a sonar return -- asks for a
sensor instead, and that one does cost the world a render, correctly and
visibly.
"""
import time

import numpy as np

import biguasim
from biguasim.agents import AgentDefinition
from biguasim.client.interpolation import PoseBuffer
from biguasim.client.remote import RemoteWorld
from biguasim.sensors import SensorDefinition

#: Minimal sensor set for a puppet. It computes nothing; the environment just
#: expects agents to have somewhere to put state.
_PUPPET_SENSORS = [{"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
                    "configuration": {"UseCOM": True, "UseRPY": False}}]


class Viewer:
    """A local, drawing-only mirror of a remote world.

    Args:
        scenario_cfg (:obj:`dict`): The package and world to open locally. Must
            match the world being watched, which the connection checks.
        address (:obj:`str`, optional): Where the world is.
        port (:obj:`int`, optional): Its request port.
        delay (:obj:`float`, optional): Snapshot intervals to draw behind.
            Defaults to 1.5.
        client_id (:obj:`str`, optional): How to identify to the world.
        ipv6 (:obj:`bool`, optional): Allow IPv6. Defaults to True.
        **make_kwargs: Passed to :func:`biguasim.make` for the local engine.
    """

    def __init__(self, scenario_cfg, address="127.0.0.1", port=8770,
                 delay=1.5, client_id=None, ipv6=True, **make_kwargs):
        local = dict(scenario_cfg)
        # Puppets are created as the roster is discovered, so nothing is
        # assumed about who is in the world when the viewer opens.
        local["agents"] = []
        local["main_agent"] = ""
        self._scenario = local

        make_kwargs.setdefault("show_viewport", True)
        self._env = biguasim.make(scenario_cfg=local, **make_kwargs)
        self._env.reset()
        # The roster changes, so the viewer always wants the full-state view.
        self._env._default_state_fn = self._env._get_full_state

        self._remote = RemoteWorld(address=address, port=port,
                                   client_id=client_id or "viewer",
                                   scenario_cfg=scenario_cfg, ipv6=ipv6)
        self._buffer = PoseBuffer(delay=delay)
        self._puppets = {}          # agent name -> vehicle type, as drawn
        self._types = {}            # agent name -> vehicle type, as announced
        self._last_tick = -1

    @property
    def remote(self):
        """:class:`~biguasim.client.remote.RemoteWorld`: The connection."""
        return self._remote

    @property
    def puppets(self):
        """:obj:`dict`: Agents currently drawn, by name and type."""
        return dict(self._puppets)

    @property
    def tick(self):
        """:obj:`int`: The newest world tick this viewer has seen."""
        return self._last_tick

    def look_at(self, target, offset=(-4.0, -4.0, 2.0)):
        """Place the camera once, aimed at a point in the world.

        Deliberately one-shot. An earlier version re-aimed every frame to track
        an agent, which meant the viewport was overwritten sixty times a second
        and nobody could move the camera by hand. Placing it once and then
        leaving it alone is what a viewer actually wants: a useful starting
        vantage point, and then control.

        Purely local either way -- the world is never told where anyone is
        looking, which is why watching costs it nothing.

        Args:
            target (sequence of :obj:`float`): The point to look at.
            offset (sequence of :obj:`float`, optional): Where to put the camera
                relative to the target.
        """
        target = np.asarray(target, dtype=np.float64)
        camera = target + np.asarray(offset, dtype=np.float64)

        direction = target - camera
        flat = float(np.hypot(direction[0], direction[1]))
        rotation = [
            0.0,
            float(np.degrees(np.arctan2(direction[2], flat))),   # pitch
            float(np.degrees(np.arctan2(direction[1], direction[0]))),  # yaw
        ]
        self._env.move_viewport(camera.tolist(), rotation)

    def connect(self):
        """Join the world and start listening for state.

        Returns:
            :obj:`dict`: The world's greeting.
        """
        info = self._remote.connect()
        self._remote.watch_state()
        return info

    # ----------------------------------------------------------------- feed

    def pump(self, budget=0.0):
        """Take whatever state has arrived.

        Args:
            budget (:obj:`float`, optional): Seconds to wait for a first
                message. Zero returns immediately if nothing is there.

        Returns:
            :obj:`int`: Snapshots taken in.
        """
        taken = 0
        timeout = budget
        while True:
            got = self._remote.recv(timeout)
            timeout = 0.0
            if got is None:
                return taken
            topic, message = got
            if not topic.startswith("state"):
                continue
            self.feed(message)
            taken += 1

    def feed(self, message):
        """Accept one state message.

        Split out from :meth:`pump` so a viewer can be driven from a recording,
        or from a test, without a world on the other end.

        Args:
            message (:obj:`dict`): A published state message.
        """
        self._last_tick = max(self._last_tick, message.get("tick", -1))
        agents = message.get("agents", {})
        # Type is not a pose and must not be interpolated, so it is kept here
        # rather than pushed through the buffer, which drops anything it cannot
        # blend between two snapshots.
        for name, pose in agents.items():
            kind = pose.get("type")
            if kind:
                self._types[name] = kind
        self._buffer.push(message.get("time", 0.0), agents)

    # ---------------------------------------------------------------- draw

    def draw(self):
        """Move every puppet to where the world says it is, and render once.

        Returns:
            :obj:`int`: How many agents were drawn.
        """
        poses = self._buffer.sample()

        for name in list(self._puppets):
            if name not in poses:
                self._retire(name)

        for name, pose in poses.items():
            if name not in self._puppets:
                self._add_puppet(name, self._types.get(name, ""))
            agent = self._env.agents.get(name + "-id0")
            if agent is None:
                continue
            # Teleported, not simulated. The world already decided this.
            agent.set_physics_state(
                location=list(pose["position"]),
                rotation=self._rotation_from(pose),
                velocity=list(pose["velocity"]),
                angular_velocity=[0.0, 0.0, 0.0])

        self._env.tick()
        return len(poses)

    @staticmethod
    def _rotation_from(pose):
        """Roll, pitch and yaw for the engine, from the published quaternion."""
        import numpy as np

        x, y, z, w = (float(v) for v in pose["quaternion"])
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll = np.degrees(np.arctan2(sinr, cosr))

        sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
        pitch = np.degrees(np.arcsin(sinp))

        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.degrees(np.arctan2(siny, cosy))
        return [float(roll), float(pitch), float(yaw)]

    def _add_puppet(self, name, agent_type):
        """Create a local stand-in for an agent the world has announced.

        The type comes from the roster rather than being guessed: a viewer
        cannot draw a vehicle without knowing which one it is. Falling back to a
        quadrotor is a last resort for a world that did not say.
        """
        full = name + "-id0"
        if full in self._env.agents:
            self._puppets[name] = agent_type
            return

        kind = agent_type or "DjiMatrice"
        sensors = [
            SensorDefinition(agent_name=full, agent_type=kind,
                             sensor_name=spec["sensor_type"],
                             sensor_type=spec["sensor_type"],
                             socket=spec.get("socket", ""),
                             config=spec.get("configuration"), tick_every=1)
            for spec in _PUPPET_SENSORS
        ]
        self._env.add_agent(AgentDefinition(
            agent_name=full, agent_type=kind, sensors=sensors))
        self._puppets[name] = agent_type

    def _retire(self, name):
        """Park a puppet whose original has gone.

        The local engine has no despawn either, so it goes out of range for the
        same reason and at the same cost as on the world side.
        """
        agent = self._env.agents.get(name + "-id0")
        if agent is not None:
            agent.set_physics_state(location=[0.0, 0.0, -100000.0],
                                    rotation=[0.0, 0.0, 0.0],
                                    velocity=[0.0, 0.0, 0.0],
                                    angular_velocity=[0.0, 0.0, 0.0])
        self._puppets.pop(name, None)

    # ----------------------------------------------------------------- run

    def run(self, seconds=None, fps=60.0, report_every=None):
        """Watch until stopped.

        Ends quietly if the local engine goes away -- closing the render window
        is a normal way to stop watching, not an error worth a stack trace.

        Args:
            seconds (:obj:`float`, optional): Stop after this long.
            fps (:obj:`float`, optional): Frames to draw per second. Drawing
                faster than snapshots arrive is the point -- that is what the
                interpolation is for.
            report_every (:obj:`float`, optional): Seconds between progress
                lines naming what is being drawn.
        """
        frame = 1.0 / fps if fps else 0.0
        deadline = None if seconds is None else time.time() + seconds
        next_report = time.time() + (report_every or 0)
        frames = 0

        while deadline is None or time.time() < deadline:
            started = time.time()
            try:
                self.pump()
                drawn = self.draw()
            except Exception as exc:                          # noqa: BLE001
                print("stopped watching: {}: {}".format(type(exc).__name__, exc))
                return
            frames += 1

            if report_every and started >= next_report:
                next_report = started + report_every
                print("  tick {}  drawing {}  ({} frames)".format(
                    self.tick, sorted(self._puppets) or "nothing", frames),
                    flush=True)

            slack = frame - (time.time() - started)
            if slack > 0:
                time.sleep(slack)

    def close(self):
        """Leave the world and shut the local engine down."""
        self._remote.close()
        self._env.__exit__(None, None, None)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()
        return False
