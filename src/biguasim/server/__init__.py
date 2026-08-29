"""Running BiguaSim as a persistent world rather than a script.

A :class:`~biguasim.server.world.World` owns one environment and advances it
tick by tick, applying whatever actions have arrived from however many clients.
Unlike :func:`biguasim.make` used directly, the world outlives any one client:
agents and sensors come and go while it runs.
"""
from biguasim.server.actions import (
    Action,
    AddSensor,
    KillAgent,
    RemoveSensor,
    RotateSensor,
    SetControl,
    SetDayTime,
    SetFogDensity,
    SetControlDefaults,
    SetWeather,
    SpawnAgent,
    decode,
    encode,
)
from biguasim.server.world import World, WorldError

__all__ = [
    "Action", "AddSensor", "KillAgent", "RemoveSensor", "RotateSensor",
    "SetControl", "SetControlDefaults", "SetDayTime", "SetFogDensity",
    "SetWeather", "SpawnAgent", "decode", "encode", "World", "WorldError",
]
