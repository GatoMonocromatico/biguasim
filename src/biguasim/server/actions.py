"""Things a client can ask the world to do.

Every action carries the tick it is meant for and enough identity to break ties
the same way twice. Ordering is `(target_tick, client_id, seq)` and never
arrival order: two clients whose messages cross on the wire must produce the
same world on a replay as they did live, and arrival order is exactly the part
that will not reproduce.

Actions come in two kinds, because they need different delivery guarantees:

* **World-mutating** -- spawning, killing, attaching sensors, weather. Losing
  one changes the world permanently, so they are acknowledged and logged.
* **Control** -- what an agent is being told to do. Latest wins, and losing one
  is harmless: the previous command stays in force, which is what a real
  vehicle does between messages anyway.
"""
from dataclasses import dataclass, field, fields
from typing import Any, Dict, List, Optional, Tuple, get_origin

_REGISTRY: Dict[str, type] = {}


def _action(cls):
    """Register an action so it can survive a round trip through the wire."""
    assert cls.kind not in _REGISTRY, "duplicate action kind: " + cls.kind
    _REGISTRY[cls.kind] = cls
    return cls


@dataclass(frozen=True, kw_only=True)
class Action:
    """Base for everything a client can submit.

    Attributes:
        client_id (:obj:`str`): Who submitted it. Also the ownership key.
        seq (:obj:`int`): Per-client counter, monotonic. Breaks ties between
            two actions the same client sent for the same tick.
        target_tick (:obj:`int`): The tick this was submitted *for*. The tick it
            actually ran on may be later, and that is what gets logged.
    """

    client_id: str = ""
    seq: int = 0
    target_tick: int = 0

    #: Wire name. Set by subclasses.
    kind = "action"
    #: Whether applying this changes the world itself rather than an input.
    mutates_world = True

    def __post_init__(self):
        # Tuple fields arrive as lists off the wire, since neither JSON nor
        # msgpack has a tuple. Without this an action would not compare equal to
        # the one that was sent, and comparing a replay against the original run
        # is the main thing anyone wants to do with a log.
        for f in fields(self):
            if get_origin(f.type) is tuple:
                value = getattr(self, f.name)
                if not isinstance(value, tuple):
                    object.__setattr__(self, f.name, tuple(value))

    @property
    def order_key(self) -> Tuple[int, str, int]:
        """The only ordering the world is allowed to use."""
        return (self.target_tick, self.client_id, self.seq)


@_action
@dataclass(frozen=True, kw_only=True)
class SetControl(Action):
    """Drive an agent. Held until superseded -- a zero-order hold."""

    kind = "set_control"
    mutates_world = False

    agent: str
    command: List[float]


@_action
@dataclass(frozen=True, kw_only=True)
class SetControlDefaults(Action):
    """The command an agent falls back to when its owner goes away.

    Without this, a client that disconnects leaves its agent running whatever it
    was last told, which for a quadrotor means flying off under full power.
    """

    kind = "set_control_defaults"

    agent: str
    command: List[float]


@_action
@dataclass(frozen=True, kw_only=True)
class SpawnAgent(Action):
    """Add an agent to a world that is already running."""

    kind = "spawn_agent"

    agent: str
    agent_type: str
    location: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    control_abstraction: str = "cmd_motor_speeds"
    sensors: List[Dict[str, Any]] = field(default_factory=list)
    dynamics: Dict[str, Any] = field(default_factory=dict)
    #: Client-side dynamics -- the world integrates nothing and takes poses from
    #: the owner instead. See the plan's option (b).
    externally_driven: bool = False


@_action
@dataclass(frozen=True, kw_only=True)
class SetPose(Action):
    """Place an agent whose dynamics live on the client.

    For vehicles the world does not integrate: a custom dynamics model, a
    hardware-in-the-loop rig, or a real airframe appearing in the shared world.
    The owner works out where it is and says so; the world still does collision
    and sensor simulation for it, and still decides what everyone else sees.

    Only valid for an agent spawned with ``externally_driven``. Accepting poses
    for a vehicle the world is also integrating would leave two things deciding
    where it is, and no way to say which is right.
    """

    kind = "set_pose"
    mutates_world = False

    agent: str
    position: Tuple[float, float, float]
    rotation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@_action
@dataclass(frozen=True, kw_only=True)
class KillAgent(Action):
    """Retire an agent.

    The engine has no despawn command, so this is a soft kill: sensors removed,
    dynamics stopped, dropped from the roster, and parked out of range.

    The actor therefore survives, and so do the five small shared memory blocks
    it was built with -- action, teleport flag, teleport command, control scheme
    and ocean current. They cannot be released while the engine still maps them,
    so the cost of a kill is bounded but permanent: five blocks per agent ever
    killed, and none of it grows with time. Sensor memory *is* reclaimed.
    """

    kind = "kill_agent"

    agent: str


@_action
@dataclass(frozen=True, kw_only=True)
class AddSensor(Action):
    """Attach a sensor to a running agent.

    ``sensor_name`` is assigned by the world, not the client, so two clients
    asking for a camera on the same agent cannot collide in shared memory.
    """

    kind = "add_sensor"

    agent: str
    sensor_type: str
    sensor_name: Optional[str] = None
    socket: str = ""
    location: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    config: Optional[Dict[str, Any]] = None
    #: Samples per second. Defaults to the world's tick rate. Must divide it
    #: evenly, since sampling is a tick divider rather than a real clock.
    hz: Optional[float] = None


@_action
@dataclass(frozen=True, kw_only=True)
class RemoveSensor(Action):
    """Detach a sensor. Its shared memory is released after the tick lands."""

    kind = "remove_sensor"

    agent: str
    sensor_name: str


@_action
@dataclass(frozen=True, kw_only=True)
class RotateSensor(Action):
    """Re-aim a sensor. Takes about three ticks to appear."""

    kind = "rotate_sensor"

    agent: str
    sensor_name: str
    rotation: Tuple[float, float, float]


@_action
@dataclass(frozen=True, kw_only=True)
class SetWeather(Action):
    """``'sunny'``, ``'cloudy'`` or ``'rain'``."""

    kind = "set_weather"

    weather: str


@_action
@dataclass(frozen=True, kw_only=True)
class SetDayTime(Action):
    """Hour of day, 0-23."""

    kind = "set_day_time"

    hour: int


@_action
@dataclass(frozen=True, kw_only=True)
class SetFogDensity(Action):
    """Fog density, 0-1."""

    kind = "set_fog_density"

    density: float


def encode(action: Action) -> Dict[str, Any]:
    """Flatten an action for the wire or the log.

    Args:
        action (:class:`Action`): The action to encode.

    Returns:
        :obj:`dict`: Plain data, msgpack/JSON friendly.
    """
    out = {"kind": action.kind}
    for f in fields(action):
        value = getattr(action, f.name)
        out[f.name] = list(value) if isinstance(value, tuple) else value
    return out


def decode(payload: Dict[str, Any]) -> Action:
    """Rebuild an action encoded by :func:`encode`.

    Args:
        payload (:obj:`dict`): What :func:`encode` produced.

    Returns:
        :class:`Action`: The action.

    Raises:
        ValueError: If the kind is unknown or the payload does not fit it. Both
            mean a client is speaking a different protocol version, so they are
            worth failing loudly on rather than guessing.
    """
    data = dict(payload)
    kind = data.pop("kind", None)
    cls = _REGISTRY.get(kind)
    if cls is None:
        raise ValueError("unknown action kind: {!r}".format(kind))

    known = {f.name for f in fields(cls)}
    unexpected = set(data) - known
    if unexpected:
        raise ValueError("unexpected fields for {}: {}".format(
            kind, ", ".join(sorted(unexpected))))
    try:
        return cls(**data)
    except TypeError as exc:
        raise ValueError("bad payload for {}: {}".format(kind, exc)) from exc


def kinds() -> Dict[str, type]:
    """Every registered action kind, by wire name."""
    return dict(_REGISTRY)
