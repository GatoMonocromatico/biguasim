"""A BiguaSim world that outlives its clients.

The world owns one environment and advances it tick by tick. Clients do not
drive it; they submit actions, and the world decides when each one lands. That
inversion is what makes it possible for several clients to share a simulation
without agreeing on anything except the tick number.

Two rules do most of the work:

* **Actions are applied in `(target_tick, client_id, seq)` order, never arrival
  order.** Arrival order is the one thing that will not reproduce on a replay.
* **The tick an action *ran* on is what gets recorded**, not the tick it asked
  for. Under load an action can slip; a log that records the request would
  replay a run that never happened.

There is deliberately no reset. A living world spawns and kills; a global reset
would silently pull the floor out from under every other client.
"""
import numpy as np
import torch

import biguasim
from biguasim.agents import AgentDefinition
from biguasim.command import (
    RemoveSensorCommand,
    RotateSensorCommand,
)
from biguasim.dynamics.agents import ModelsFactory
from biguasim.sensors import SensorDefinition
from biguasim.server import actions as act
from biguasim.util import gpu


#: Command width per control abstraction. ``cmd_motor_speeds`` is not here
#: because it depends on the vehicle -- it is one value per rotor, and the
#: rotor count comes from the model's own ``rotor_pos`` parameter.
COMMAND_WIDTHS = {
    "accel": 6,
    "cmd_vel": 3,
    "cmd_vel_yaw": 4,
    "cmd_pos_yaw": 4,
    "cmd_rudders_sterns_motor_speed": 5,
    "cmd_depth_heading_rpm_surge": 4,
}


def neutral_command(control_abstraction, params):
    """A do-nothing command of the right width for one agent.

    Every agent needs one from the moment it exists. The engine does not
    integrate these vehicles -- the Python dynamics model does, and the engine
    applies whatever that model produces -- so an agent that is never stepped
    is not an agent left alone. It has no forces on it at all, gravity
    included, and it hangs in the air.

    That distinction only shows up once an agent is told how its actions should
    be read: before a control scheme is set the engine falls back to simulating
    the actor itself, which looks like normal physics right up until it stops
    being.

    Args:
        control_abstraction (:obj:`str`): What the commands mean.
        params (:obj:`dict`): The model's parameters, for the rotor count.

    Returns:
        :obj:`list` of :obj:`float`: Zeros, of the width the model expects.
    """
    width = COMMAND_WIDTHS.get(control_abstraction)
    if width is None:
        # Per-rotor abstractions. Every vehicle family builds its geometry from
        # rotor_pos, keyed by rotor name, so its length is the motor count.
        rotors = params.get("rotor_pos") if isinstance(params, dict) else None
        width = len(rotors) if rotors else 4
    return [0.0] * width


class WorldError(Exception):
    """A client asked for something the world will not do."""


#: How far ahead of the current tick clients are asked to aim. Fixed on purpose:
#: adapting it to measured latency makes the same log replay differently on a
#: different network, which is the whole thing we are trying to avoid.
DEFAULT_INPUT_DELAY = 3

#: Leaves room in the engine's command buffer rather than filling it exactly.
DEFAULT_COMMAND_BUDGET = 8 * 1024 * 1024

#: How far inside the world's own bounds a parked agent sits, in metres. A
#: margin rather than the corner exactly, so nothing rounds its way back out.
GRAVEYARD_INSET = 1.0

def graveyard(env_min, inset=GRAVEYARD_INSET):
    """Where to park a soft-killed agent in a world with these bounds.

    The engine has no despawn, so a killed actor lingers and the most that can
    be done is put it where nothing operates. Where that is has to be derived
    rather than fixed. Every world declares its own ``env_min``/``env_max`` box
    -- handed to the engine as ``-EnvMinZ`` and friends, see
    :class:`biguasim.environments.BiguaSimEnvironment` -- and a teleport outside
    that box is not refused, it is ignored, leaving the vehicle exactly where it
    died. So a constant that happened to suit one world did nothing whatsoever
    in every other, and did it silently.

    The far bottom corner is as far from the action as a bounded world allows,
    and is usually under the terrain.

    .. warning::

       This is not out of range of anything, and in a small world nothing is.
       ``CompetionMap`` is 400 x 400 x 100 m; its corner is under 300 m from the
       middle. A camera pointed that way sees what died there, the octree still
       carries it, and a vehicle flown into the corner can still hit it. Killing
       an agent removes it from the roster, not from the world.

    Args:
        env_min (sequence of :obj:`float`): The world's lower bound, in metres.
        inset (:obj:`float`, optional): How far inside that bound to sit.

    Returns:
        :obj:`tuple`: The ``(x, y, z)`` to park at.
    """
    return tuple(float(v) + inset for v in env_min[:3])


#: Blocks a soft-killed agent keeps: action, teleport flag, teleport command,
#: control scheme, ocean current. Retained because the engine still maps them.
RETAINED_BLOCKS_PER_KILLED_AGENT = 5


def _base(name):
    """Strip the ``-id0`` the environment appends to every agent."""
    return name.split("-id")[0]


class World:
    """A running simulation that accepts actions from many clients.

    Args:
        scenario_cfg (:obj:`dict`): Starting scenario, same shape
            :func:`biguasim.make` takes. May contain no agents at all -- the
            world is perfectly happy empty and filled in later.
        input_delay (:obj:`int`, optional): Ticks between submission and
            execution. Defaults to :data:`DEFAULT_INPUT_DELAY`.
        command_budget (:obj:`int`, optional): Bytes of engine command buffer to
            use per tick before deferring the rest. Defaults to
            :data:`DEFAULT_COMMAND_BUDGET`.
        admin_clients (:obj:`set` of :obj:`str`, optional): Clients allowed to
            act on entities they do not own.
        record (callable, optional): Called as
            ``record(applied_tick, action, error)`` for every action the world
            attempts, in the order it attempted them, with ``error`` set to the
            message if it failed. This is the whole hook the replay log needs.
        **make_kwargs: Passed through to :func:`biguasim.make`.
    """

    def __init__(self, scenario_cfg, input_delay=DEFAULT_INPUT_DELAY,
                 command_budget=DEFAULT_COMMAND_BUDGET, admin_clients=None,
                 record=None, **make_kwargs):
        make_kwargs.setdefault("show_viewport", False)
        self._scenario = scenario_cfg
        self._input_delay = int(input_delay)
        self._command_budget = int(command_budget)
        self._admin = set(admin_clients or ())
        self._record = record
        #: Set by the service when the world was started with --allow-sitl.
        #: Without one, a spawn asking for ArduPilot is refused rather than
        #: quietly producing a vehicle nothing is flying.
        self.sitl = None

        self._env = biguasim.make(scenario_cfg=scenario_cfg, **make_kwargs)
        self._env.reset()

        # Derived from this world's own bounds, because a location outside them
        # is ignored rather than refused -- see graveyard().
        self._graveyard = graveyard(self._env._env_min)

        # The environment picks its state function once, at construction, and a
        # single-agent scenario gets the one that only ever reports the main
        # agent. In a world whose roster changes that is never what is wanted:
        # anything spawned later would simply not appear in the state.
        self._env._default_state_fn = self._env._get_full_state

        self._tick = 0
        self._pending = []          # submitted, not yet due
        self._deferred = []         # due, but the command buffer was full
        self._controls = {}         # base agent name -> current command
        self._defaults = {}         # base agent name -> fallback command
        self._owner = {}            # entity key -> client id
        self._external = set()      # agents whose dynamics live client-side
        self._types = {}            # agent -> vehicle type, so viewers know
        self._sensor_seq = 0
        self._errors = []           # (tick, action, message), drained by the caller
        self._corrections = []      # (tick, agent, pose) for client-driven agents

        self._device = torch.device(
            "cuda:" + str(gpu()) if torch.cuda.is_available() else "cpu")

        for name in list(self._env._dynamics_dict):
            # Same reasoning as a spawned agent: uncommanded is not the same as
            # unforced, and an agent nobody has driven yet still has to fall.
            model = self._env._dynamics_dict[name]
            self._controls.setdefault(name, neutral_command(
                getattr(model, "control_abstraction", ""),
                getattr(model, "_params", {})))
        for spec in scenario_cfg.get("agents", []):
            self._types[_base(spec.get("agent_name", ""))] = spec.get("agent_type", "")

    # ---------------------------------------------------------------- state

    @property
    def tick(self):
        """:obj:`int`: Ticks executed so far."""
        return self._tick

    @property
    def next_tick(self):
        """:obj:`int`: The earliest tick a client should aim an action at."""
        return self._tick + self._input_delay

    @property
    def ticks_per_sec(self):
        """:obj:`int`: How many ticks make a simulated second.

        Reported to clients on connect. A sensor rate has to divide it, and a
        flight controller has to live with it, so a client that had to be told
        this number separately would be a client that could be told the wrong
        one.
        """
        return self._env._ticks_per_sec

    @property
    def agents(self):
        """:obj:`list` of :obj:`str`: Live agents, by base name."""
        return sorted(self._controls)

    @property
    def graveyard(self):
        """:obj:`tuple`: Where this world parks soft-killed agents."""
        return self._graveyard

    @property
    def agent_types(self):
        """:obj:`dict`: Agent name to vehicle type.

        Published with the roster: a viewer drawing the world locally has to
        know which vehicle to draw before it can draw anything.
        """
        return {name: self._types.get(name, "") for name in self._controls}

    def owner_of(self, agent):
        """Who owns an agent, or ``None`` if it came from the scenario.

        Args:
            agent (:obj:`str`): Base agent name.

        Returns:
            :obj:`str` or None: The owning client id.
        """
        return self._owner.get(("agent", _base(agent)))

    # --------------------------------------------------------------- submit

    def submit(self, action):
        """Queue an action.

        An action aimed at a tick that has already run cannot be honoured, so it
        is moved to the next available tick rather than dropped -- the client's
        intent survives, and the log records where it actually landed.

        Args:
            action (:class:`~biguasim.server.actions.Action`): The action.

        Returns:
            :obj:`int`: The tick it is now scheduled for.

        Raises:
            WorldError: If the client may not act on that entity.
        """
        self._authorize(action)

        earliest = self._tick + 1
        if action.target_tick < earliest:
            action = type(action)(**{
                **{f: getattr(action, f) for f in action.__dataclass_fields__},
                "target_tick": earliest,
            })

        self._pending.append(action)
        return action.target_tick

    def preload(self, action):
        """Queue an action at exactly the tick it names, checking nothing.

        For replay, which is re-running a script the world already accepted
        once. :meth:`submit` would move a stale tick forward and re-check
        ownership, both of which would make the replay differ from the run it
        is supposed to reproduce.

        Args:
            action (:class:`~biguasim.server.actions.Action`): The action.
        """
        self._pending.append(action)

    def _authorize(self, action):
        """Refuse an action a client is not entitled to submit.

        Checked here, at submit time, rather than when the action runs: whether
        a client *may* act is knowable the moment it asks, and answering
        immediately gives it something it can act on. Whether the action works
        is a separate question with a separate answer, ticks later.

        Agents from the starting scenario are owned by nobody and open to all --
        they are the world's furniture rather than anyone's property.
        """
        agent = getattr(action, "agent", None)
        if agent is None or action.client_id in self._admin:
            return
        if isinstance(action, act.SpawnAgent):
            return

        owner = self._owner.get(("agent", _base(agent)))
        if owner is not None and owner != action.client_id:
            raise WorldError(
                "client {!r} may not act on agent {!r} (owned by {!r})".format(
                    action.client_id, agent, owner))

    def release_client(self, client_id):
        """Hand back everything a departed client owned.

        Agents fall back to the command registered with
        :class:`~biguasim.server.actions.SetControlDefaults`, or are soft-killed
        if none was. Leaving them on their last command would mean a quadrotor
        whose pilot's laptop closed keeps climbing under full power.

        Args:
            client_id (:obj:`str`): The departing client.

        Returns:
            :obj:`list` of :obj:`str`: Agents that were killed outright.
        """
        killed = []
        for (kind, *rest), owner in list(self._owner.items()):
            if owner != client_id or kind != "agent":
                continue
            agent = rest[0]
            if agent in self._defaults:
                self._controls[agent] = list(self._defaults[agent])
                self._owner.pop(("agent", agent), None)
            else:
                self._kill_agent(agent)
                killed.append(agent)
        return killed

    # ----------------------------------------------------------------- tick

    def step(self):
        """Apply everything due and advance the world one tick.

        Returns:
            :obj:`dict`: The environment state for this tick, as
            :meth:`biguasim.environments.BiguaSimEnvironment.tick` returns it.
        """
        for action in self._due():
            self._apply(action)

        state = self._advance()
        self._tick += 1
        return state

    def _due(self):
        """Actions to apply this tick, in the only order that reproduces.

        Deferred actions sort in naturally: they keep their original key, so a
        slipped action stays behind whatever was already ahead of it.
        """
        ready, still_pending = [], []
        for action in self._pending:
            (ready if action.target_tick <= self._tick else still_pending).append(action)
        self._pending = still_pending

        ready.extend(self._deferred)
        self._deferred = []
        ready.sort(key=lambda a: a.order_key)
        return ready

    def _apply(self, action):
        """Apply one action, or defer it if the command buffer is full."""
        if action.mutates_world:
            center = self._env._command_center
            if center.pending_bytes >= self._command_budget:
                self._deferred.append(action)
                return

        handler = self._HANDLERS.get(type(action))
        if handler is None:
            raise WorldError("no handler for {}".format(type(action).__name__))

        # A client asking for something impossible -- driving an agent that was
        # killed a moment ago, say -- is ordinary traffic in a shared world, not
        # grounds for stopping it. The failure goes back to whoever asked.
        #
        # Every exception is caught, not just WorldError. A handler reaches into
        # the environment and the engine, so it can raise anything at all, and a
        # world that dies because one client asked for an agent type this build
        # does not have is a world that any client can kill by accident. What
        # the failure was matters to the client that caused it; that it happened
        # at all must not matter to everyone else.
        error = None
        try:
            handler(self, action)
        except Exception as exc:                              # noqa: BLE001
            error = ("{}: {}".format(type(exc).__name__, exc)
                     if not isinstance(exc, WorldError) else str(exc))
            self._errors.append((self._tick, action, error))

        if self._record is not None:
            self._record(self._tick, action, error)

    def _advance(self):
        """Tick the engine, then drive whatever has a command.

        This is what :meth:`biguasim.environments.BiguaSimEnvironment.step`
        does, unrolled. The world needs it unrolled because ``step`` insists on
        a command for every model it knows about -- so a newly spawned agent
        nobody has commanded yet, or an agent whose dynamics live on the client,
        would make it raise. Iteration follows ``_dynamics_dict`` insertion
        order, which is stable, so this is no less reproducible than ``step``.
        """
        state = self._env.tick()
        dt = state["t"]

        if self._external:
            self._collect_corrections(state)

        for name, model in list(self._env._dynamics_dict.items()):
            command = self._controls.get(name)
            if command is None or name not in state:
                continue
            action = model.step(state[name], [list(command)], dt)
            engine_agent = self._env.agents.get(name + "-id0")
            if engine_agent is not None:
                engine_agent.act(action[0])

        return state

    # ------------------------------------------------------------- handlers

    def _set_pose(self, action):
        """Place a client-driven agent where its owner says it is."""
        agent = _base(action.agent)
        if agent not in self._external:
            raise WorldError(
                "agent {!r} is driven by the world; use set_control".format(agent))

        engine_agent = self._env.agents.get(agent + "-id0")
        if engine_agent is None:
            raise WorldError("no such agent: {!r}".format(action.agent))

        engine_agent.set_physics_state(
            location=list(action.position),
            rotation=list(action.rotation),
            velocity=list(action.velocity),
            angular_velocity=list(action.angular_velocity))

    def _collect_corrections(self, state):
        """Tell owners when the world disagrees with them about a collision.

        A client integrating its own vehicle does not know about the world's
        geometry, so it will happily fly through a pier. The world is
        authoritative on contact and says so; what the client does about it --
        accept the correction, blend towards it, ignore it -- is the client's
        business, and deliberately not decided here.
        """
        for agent in self._external:
            frames = state.get(agent)
            if not frames:
                continue
            collision = frames[0].get("CollisionSensor")
            if collision is None or not np.any(np.asarray(collision)):
                continue
            dynamics = frames[0].get("DynamicsSensor")
            if dynamics is None:
                continue
            values = np.asarray(dynamics, dtype=np.float64)
            self._corrections.append((self._tick, agent, {
                "position": values[6:9].tolist(),
                "velocity": values[3:6].tolist(),
                "quaternion": values[15:19].tolist(),
            }))

    def drain_corrections(self):
        """Take collision corrections raised since this was last called.

        Returns:
            :obj:`list`: ``(tick, agent, pose)`` triples, oldest first.
        """
        found, self._corrections = self._corrections, []
        return found

    def _set_control(self, action):
        """Set an agent's standing command.

        Held until superseded, so a client that stops sending does not cause the
        vehicle to stop being driven -- which is how a real autopilot behaves
        between messages, and why dropping one of these is harmless.
        """
        agent = _base(action.agent)
        if agent not in self._controls:
            raise WorldError("no such agent: {!r}".format(action.agent))
        if agent in self._external:
            raise WorldError(
                "agent {!r} is driven by its client; use set_pose".format(agent))
        self._controls[agent] = list(action.command)

    def _set_control_defaults(self, action):
        """Record what an agent should do if its owner disappears.

        Without one, :meth:`release_client` has no safe option but to kill the
        agent, because leaving it on its last command means a quadrotor whose
        pilot closed their laptop climbs at full power until it leaves the map.
        """
        self._defaults[_base(action.agent)] = list(action.command)

    def _spawn_agent(self, action):
        """Add an agent to the running world.

        The environment appends ``-id0`` to every agent name and keys its own
        dictionaries on the result, while dynamics and published state use the
        base name. Both are maintained here.

        The dynamics model is built the same way :mod:`biguasim.environments`
        builds one at load time, because an agent spawned into a running world
        should be indistinguishable from one that started in it.
        """
        agent = _base(action.agent)
        if agent in self._controls:
            raise WorldError("agent already exists: {!r}".format(agent))

        if action.ardupilot is not None:
            return self._provision_ardupilot(agent, action)

        full_name = agent + "-id0"
        sensors = [
            SensorDefinition(
                agent_name=full_name,
                agent_type=action.agent_type,
                sensor_name=spec.get("sensor_name", spec["sensor_type"]),
                sensor_type=spec["sensor_type"],
                socket=spec.get("socket", ""),
                location=spec.get("location", (0, 0, 0)),
                rotation=spec.get("rotation", (0, 0, 0)),
                config=spec.get("configuration"),
                # Spawn-time sensors get the same rate divider a later
                # AddSensor would. Without this a camera declared at 10 Hz
                # renders every tick -- at 200 ticks/sec that is twenty times
                # the intended load, paid by every client in the world.
                tick_every=self._tick_every(spec.get("Hz")),
            )
            for spec in action.sensors
        ]

        model_cls = ModelsFactory.build_model(action.agent_type)
        params = model_cls._params.copy()
        for key in params:
            if key in action.dynamics:
                params[key] = action.dynamics[key]

        self._env.add_agent(AgentDefinition(
            agent_name=full_name,
            agent_type=action.agent_type,
            sensors=sensors,
            starting_loc=tuple(action.location),
            starting_rot=tuple(action.rotation),
        ))

        # The engine has to be told how to read this agent's actions, exactly
        # as environments.py does immediately after its own add_agent. Every
        # fresh agent starts on scheme 0 (agents.py), while a quadrotor's model
        # emits scheme 1 and the underwater models scheme 2 -- so without this
        # the engine interprets thrust as something else entirely. The vehicle
        # then spawns, falls, rests on the ground and answers no command, with
        # nothing anywhere reporting an error.
        engine_agent = self._env.agents.get(full_name)
        if engine_agent is not None:
            engine_agent.set_control_scheme(model_cls._scheme)

        if action.externally_driven:
            # Option (b): the owner integrates this vehicle and sends poses.
            # The world still does collision and sensors for it.
            self._external.add(agent)
            self._controls[agent] = None
        else:
            self._env._dynamics_dict[agent] = model_cls(
                batch_size=1,
                device=self._device,
                control_abstraction=action.control_abstraction,
                params=params,
            )
            # Not None. An agent with no command is skipped in _advance, so its
            # model never runs and the engine -- which is now told to take its
            # motion from that model -- applies nothing at all. It would hang
            # in the air until somebody happened to command it.
            self._controls[agent] = neutral_command(
                action.control_abstraction, params)

        self._types[agent] = action.agent_type
        if action.client_id:
            self._owner[("agent", agent)] = action.client_id

    def provision_ardupilot(self, action):
        """Start a flight controller for an agent, outside the tick loop.

        Called straight from the transport rather than queued, because it
        creates nothing in the world -- it starts two processes beside it --
        and the caller needs the ports back in the same reply. The spawn that
        does reach the world is the pilot's, and that one is ordered like
        every other action.

        Args:
            action (:class:`~biguasim.server.actions.SpawnAgent`): The request,
                with its ``ardupilot`` block set.

        Returns:
            :obj:`dict`: Instance, ports and the command that was run.

        Raises:
            WorldError: If the world will not do it.
        """
        agent = _base(action.agent)
        if agent in self._controls:
            raise WorldError("agent already exists: {!r}".format(agent))
        return self._provision_ardupilot(agent, action)

    def _provision_ardupilot(self, agent, action):
        """Start a flight controller for this vehicle, and let it do the spawn.

        Deliberately does not create the agent. The pilot does, once its SITL
        has connected, because a served world free-runs: an agent that exists
        before something is stabilising it is integrated on whatever command it
        has and falls out of the sky while the controller boots.

        Returns:
            :obj:`dict`: The ports the caller needs, chiefly the MAVLink one to
            point a GCS at.
        """
        if self.sitl is None or not self.sitl.enabled:
            raise WorldError(
                "this world does not start flight controllers: it was started "
                "without --allow-sitl")
        try:
            entry = self.sitl.provision(agent, action.agent_type, action.ardupilot)
        except Exception as exc:                                  # noqa: BLE001
            raise WorldError(str(exc))

        # Recorded against the requesting client, not the pilot that will do the
        # spawning, so whoever asked for the vehicle is who can retire it.
        if action.client_id:
            self._owner[("sitl", agent)] = action.client_id
        return entry

    @staticmethod
    def _removal_def(engine_agent, full_name, sensor_name):
        """A SensorDefinition good enough to detach an existing sensor.

        remove_sensors only reads the name, but SensorDefinition validates the
        type on construction, so the live sensor is asked what it is.
        """
        sensor = engine_agent.sensors[sensor_name]
        return SensorDefinition(
            agent_name=full_name,
            agent_type=getattr(engine_agent, "agent_type", ""),
            sensor_name=sensor_name,
            sensor_type=type(sensor).sensor_type,
            existing=True,
        )

    def _kill_agent(self, agent_or_action):
        """Retire an agent as far as the engine allows.

        Sensors go, and their shared memory with them. The agent's own five
        blocks stay: the engine still owns the actor, so unlinking them would
        hand it an orphaned inode. See KillAgent for the accounting.
        """
        agent = _base(getattr(agent_or_action, "agent", agent_or_action))
        if agent not in self._controls:
            raise WorldError("no such agent: {!r}".format(agent))

        full_name = agent + "-id0"
        engine_agent = self._env.agents.get(full_name)
        if engine_agent is not None:
            for sensor_name in list(engine_agent.sensors):
                engine_agent.remove_sensors(
                    self._removal_def(engine_agent, full_name, sensor_name))
            engine_agent.clear_action()
            # No despawn command exists, so the actor is parked instead. It
            # still costs the engine collision and render.
            engine_agent.set_physics_state(
                location=list(self._graveyard), rotation=[0, 0, 0],
                velocity=[0, 0, 0], angular_velocity=[0, 0, 0])

        if self.sitl is not None:
            # Otherwise the SITL and pilot keep running, holding their ports and
            # trying to fly an agent the world no longer has.
            self.sitl.release(agent)

        self._env._dynamics_dict.pop(agent, None)
        self._controls.pop(agent, None)
        self._defaults.pop(agent, None)
        self._external.discard(agent)
        self._types.pop(agent, None)
        self._owner.pop(("agent", agent), None)

    def _add_sensor(self, action):
        """Attach a sensor to a running agent, under a name the world chooses.

        Returns:
            :obj:`str`: The name actually used.
        """
        agent = _base(action.agent)
        full_name = agent + "-id0"
        engine_agent = self._env.agents.get(full_name)
        if engine_agent is None:
            raise WorldError("no such agent: {!r}".format(action.agent))

        # Names are assigned here, never taken from the client: the shared
        # memory key is derived from agent + sensor name, so two clients asking
        # for "RGBCamera" on one agent would otherwise collide in /dev/shm.
        name = action.sensor_name
        if name is None or name in engine_agent.sensors:
            self._sensor_seq += 1
            name = "{}_{}_{}".format(
                action.sensor_type, action.client_id or "anon", self._sensor_seq)

        engine_agent.add_sensors(SensorDefinition(
            agent_name=full_name,
            agent_type=getattr(engine_agent, "agent_type", ""),
            sensor_name=name,
            sensor_type=action.sensor_type,
            socket=action.socket,
            location=tuple(action.location),
            rotation=tuple(action.rotation),
            config=action.config,
            tick_every=self._tick_every(action.hz),
        ))
        self._owner[("sensor", agent, name)] = action.client_id
        return name

    def _tick_every(self, hz):
        """Turn a sample rate into a tick divider.

        Args:
            hz (:obj:`float` or None): Samples per second, or None for every tick.

        Returns:
            :obj:`int`: Ticks between samples.

        Raises:
            WorldError: If the rate is faster than the world ticks, or does not
                divide the tick rate evenly -- sampling is a divider, not a
                clock, so anything else would silently round.
        """
        if hz is None:
            return 1
        rate = self._env._ticks_per_sec
        if hz <= 0 or hz > rate:
            raise WorldError(
                "sensor rate {} Hz is outside 0 < hz <= {} (the tick rate)".format(hz, rate))
        every = rate / hz
        if int(every) != every:
            raise WorldError(
                "sensor rate {} Hz does not divide the {} Hz tick rate evenly".format(hz, rate))
        return int(every)

    def _remove_sensor(self, action):
        """Detach a sensor.

        The shared memory behind it is released at the end of the tick rather
        than now -- see :meth:`biguasim.biguasimclient.BiguaSimClient.free`.
        """
        agent = _base(action.agent)
        engine_agent = self._env.agents.get(agent + "-id0")
        if engine_agent is None or action.sensor_name not in engine_agent.sensors:
            raise WorldError("no such sensor: {!r} on {!r}".format(
                action.sensor_name, action.agent))
        engine_agent.remove_sensors(
            self._removal_def(engine_agent, agent + "-id0", action.sensor_name))
        self._owner.pop(("sensor", agent, action.sensor_name), None)

    def _rotate_sensor(self, action):
        """Re-aim a sensor. The engine applies it within about three ticks."""
        self._env._enqueue_command(RotateSensorCommand(
            _base(action.agent) + "-id0", action.sensor_name, list(action.rotation)))

    def _set_weather(self, action):
        """Change the weather.

        Weather is entirely command-driven, so replaying the same actions into a
        viewer's local world syncs its weather for free.
        """
        self._env.weather.set_weather(action.weather)

    def _set_day_time(self, action):
        """Set the hour of day, 0-23."""
        self._env.weather.set_day_time(action.hour)

    def _set_fog_density(self, action):
        """Set fog density, 0-1."""
        self._env.weather.set_fog_density(action.density)

    _HANDLERS = {
        act.SetControl: _set_control,
        act.SetPose: _set_pose,
        act.SetControlDefaults: _set_control_defaults,
        act.SpawnAgent: _spawn_agent,
        act.KillAgent: _kill_agent,
        act.AddSensor: _add_sensor,
        act.RemoveSensor: _remove_sensor,
        act.RotateSensor: _rotate_sensor,
        act.SetWeather: _set_weather,
        act.SetDayTime: _set_day_time,
        act.SetFogDensity: _set_fog_density,
    }

    def stop_flight_controllers(self):
        """End every SITL and pilot this world started."""
        if self.sitl is not None:
            self.sitl.close()

    def drain_errors(self):
        """Take the actions that failed since this was last called.

        Each is a ``(tick, action, message)`` triple. The transport turns these
        into replies to the client that submitted them; nothing else needs them.

        Returns:
            :obj:`list`: The failures, oldest first.
        """
        failures, self._errors = self._errors, []
        return failures

    # ---------------------------------------------------------------- close

    def close(self):
        """Shut the world down, and everything it started."""
        self.stop_flight_controllers()
        self._env.__exit__(None, None, None)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
