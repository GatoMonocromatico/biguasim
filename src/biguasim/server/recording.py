"""Recording a world, and playing it back by re-running it.

Because the simulator is reproducible (tools/determinism_probe.py, and the
tests beside it), a recording does not have to store what happened. It stores
what was *asked for*, and running the same asks through the same world produces
the same run again. That makes recordings tiny, and -- much more useful -- it
makes them editable: change an action, re-run, see what would have happened.

Keyframes are written alongside anyway, every so often. They are not needed for
correctness here; they exist so playback can jump to minute forty without
simulating the first thirty-nine, and so a replay that has drifted says so
instead of quietly diverging.

The one thing that must be pinned is the compute device. cpu and cuda agree
only to float64 rounding, and the environment picks cuda when it can, choosing
between GPUs by free VRAM -- so the device can change between runs on its own.
A recording says which one it was, and replay refuses to pretend otherwise.

The format is length-prefixed msgpack: a header, then records in the order they
happened. Deliberately dull and dependency-free; converting to MCAP later is a
transcoding job, not a redesign.
"""
import struct
import time

import msgpack
import numpy as np
import torch

from biguasim.server import actions as act
from biguasim.server import protocol as proto

#: Bumped when the record shapes change incompatibly.
FORMAT_VERSION = 1

_HEADER = "header"
_ACTION = "action"
_KEYFRAME = "keyframe"

_LEN = struct.Struct("<I")


def current_device():
    """The device the dynamics will actually run on.

    Mirrors the choice in :mod:`biguasim.environments` so a recording can say
    what it used.

    Returns:
        :obj:`str`: e.g. ``'cuda:0'`` or ``'cpu'``.
    """
    if torch.cuda.is_available():
        from biguasim.util import gpu
        return "cuda:" + str(gpu())
    return "cpu"


def _write(handle, record):
    payload = msgpack.packb(record, use_bin_type=True)
    handle.write(_LEN.pack(len(payload)))
    handle.write(payload)


def _read_all(handle):
    while True:
        size = handle.read(_LEN.size)
        if not size or len(size) < _LEN.size:
            return
        payload = handle.read(_LEN.unpack(size)[0])
        yield msgpack.unpackb(payload, raw=False, strict_map_key=False)


class Recorder:
    """Writes a replayable log of a world.

    Args:
        path (:obj:`str`): Where to write.
        scenario_cfg (:obj:`dict`): The scenario the world was built from.
            Stored whole, so a replay can rebuild the same world.
        keyframe_every (:obj:`int`, optional): Ticks between keyframes. Zero
            disables them. Defaults to 200.
        device (:obj:`str`, optional): Overrides the detected device.
    """

    def __init__(self, path, scenario_cfg, keyframe_every=200, device=None):
        self._path = path
        self._handle = open(path, "wb")
        self._keyframe_every = int(keyframe_every)
        self._actions = 0
        self._keyframes = 0

        _write(self._handle, {
            "type": _HEADER,
            "format": FORMAT_VERSION,
            "protocol": proto.PROTOCOL_VERSION,
            "build": proto.build_id(scenario_cfg),
            "scenario": scenario_cfg,
            "device": device or current_device(),
            "keyframe_every": self._keyframe_every,
            "created": time.time(),
        })

    @property
    def path(self):
        """:obj:`str`: Where this is being written."""
        return self._path

    @property
    def counts(self):
        """:obj:`tuple`: How many actions and keyframes have been written."""
        return self._actions, self._keyframes

    def record_action(self, tick, action, error=None):
        """Note that the world attempted an action. Matches ``World``'s hook.

        The tick recorded is the one it *ran* on, which under load is not the
        one it asked for. Recording the request would replay a run that never
        happened.

        Args:
            tick (:obj:`int`): The tick it ran on.
            action (:class:`~biguasim.server.actions.Action`): What ran.
            error (:obj:`str`, optional): Why it failed, if it did. Failures are
                kept: a faithful replay should fail the same way.
        """
        _write(self._handle, {
            "type": _ACTION, "tick": int(tick),
            "action": act.encode(action), "error": error,
        })
        self._actions += 1

    def observe(self, tick, state):
        """Offer a tick's state, writing a keyframe if one is due.

        Args:
            tick (:obj:`int`): The tick just completed.
            state (:obj:`dict`): The world's state for it.
        """
        if not self._keyframe_every or tick % self._keyframe_every:
            return

        agents = {}
        for name, frames in state.items():
            if name == "t" or not isinstance(frames, list) or not frames:
                continue
            dynamics = frames[0].get("DynamicsSensor")
            if dynamics is not None:
                agents[name] = np.asarray(dynamics, dtype=np.float64).tolist()

        _write(self._handle, {
            "type": _KEYFRAME, "tick": int(tick),
            "time": float(state.get("t", 0.0)), "agents": agents,
        })
        self._keyframes += 1

    def close(self):
        """Finish the file."""
        if self._handle is not None:
            self._handle.close()
            self._handle = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


class Recording:
    """A recording on disk, opened for reading.

    Args:
        path (:obj:`str`): The file to read.
    """

    def __init__(self, path):
        self.path = path
        self.header = None
        self.actions = []       # (tick, Action, error)
        self.keyframes = {}     # tick -> {agent: state vector}

        with open(path, "rb") as handle:
            for record in _read_all(handle):
                kind = record.get("type")
                if kind == _HEADER:
                    self.header = record
                elif kind == _ACTION:
                    self.actions.append((record["tick"],
                                         act.decode(record["action"]),
                                         record.get("error")))
                elif kind == _KEYFRAME:
                    self.keyframes[record["tick"]] = record["agents"]

        if self.header is None:
            raise ValueError("{}: no header; not a recording".format(path))
        if self.header.get("format") != FORMAT_VERSION:
            raise ValueError("{}: format {}, expected {}".format(
                path, self.header.get("format"), FORMAT_VERSION))

    @property
    def scenario(self):
        """:obj:`dict`: The scenario to rebuild the world from."""
        return self.header["scenario"]

    @property
    def device(self):
        """:obj:`str`: The device the original run used."""
        return self.header["device"]

    @property
    def last_tick(self):
        """:obj:`int`: The last tick anything was recorded for."""
        ticks = [t for t, _, _ in self.actions] + list(self.keyframes)
        return max(ticks) if ticks else 0

    def check_device(self, strict=True):
        """Compare the recording's device against this machine's.

        Args:
            strict (:obj:`bool`, optional): Raise rather than return a warning.

        Returns:
            :obj:`str` or None: A warning, if they differ.

        Raises:
            ValueError: If they differ and ``strict``.
        """
        here = current_device()
        if here == self.device:
            return None
        message = ("recorded on {!r}, replaying on {!r}; results agree only to "
                   "float64 rounding, so this will drift".format(self.device, here))
        if strict:
            raise ValueError(message)
        return message


def replay(recording, ticks=None, strict_device=True, **world_kwargs):
    """Re-run a recording and report where, if anywhere, it diverged.

    Args:
        recording (:class:`Recording` or :obj:`str`): The recording, or its path.
        ticks (:obj:`int`, optional): How far to run. Defaults to the end.
        strict_device (:obj:`bool`, optional): Refuse to replay a recording made
            on a different device. Defaults to True.
        **world_kwargs: Passed to :class:`~biguasim.server.world.World`.

    Returns:
        :obj:`dict`: ``matched`` keyframes, ``compared`` keyframes, and
        ``divergences`` as ``(tick, agent, max_abs_delta)`` triples.
    """
    from biguasim.server.world import World

    if isinstance(recording, str):
        recording = Recording(recording)
    recording.check_device(strict=strict_device)

    horizon = recording.last_tick if ticks is None else ticks
    scheduled = {}
    for tick, action, _error in recording.actions:
        scheduled.setdefault(tick, []).append(action)

    divergences = []
    compared = 0
    with World(recording.scenario, **world_kwargs) as world:
        for actions in scheduled.values():
            for action in actions:
                world.preload(action)

        while world.tick <= horizon:
            expected = recording.keyframes.get(world.tick)
            state = world.step()

            if expected is None:
                continue
            compared += 1
            for agent, values in expected.items():
                frames = state.get(agent)
                actual = frames[0].get("DynamicsSensor") if frames else None
                if actual is None:
                    divergences.append((world.tick - 1, agent, float("inf")))
                    continue
                delta = np.abs(np.asarray(actual, dtype=np.float64)
                               - np.asarray(values, dtype=np.float64)).max()
                if delta > 0:
                    divergences.append((world.tick - 1, agent, float(delta)))

    return {
        "compared": compared,
        "matched": compared - len({t for t, _, _ in divergences}),
        "divergences": divergences,
    }
