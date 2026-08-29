"""Smoothing the gap between snapshots.

State arrives at the world's tick rate -- twenty a second, say -- and a viewer
draws at whatever its own display manages, which is more. Something has to fill
the gap or everything visibly steps.

The fix is the one every networked game uses: draw the recent past rather than
the present. Holding back by a little over one snapshot interval means there is
almost always a snapshot on each side of the moment being drawn, so the answer
is an interpolation between two things that really happened rather than a guess
about what happens next. The cost is a fixed sliver of latency; the benefit is
that a late or dropped snapshot passes unnoticed.

This is for drawing only. Nothing here touches the sensor path, and that
separation is structural rather than a setting: a smoothed pose is a plausible
fiction, and it has no business anywhere near data somebody might use.
"""
import bisect

import numpy as np


def slerp(q0, q1, fraction):
    """Interpolate between two orientations along the shorter arc.

    Args:
        q0, q1 (:obj:`np.ndarray`): Quaternions.
        fraction (:obj:`float`): 0 gives ``q0``, 1 gives ``q1``.

    Returns:
        :obj:`np.ndarray`: The interpolated quaternion, normalised.
    """
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    dot = float(np.dot(q0, q1))

    # q and -q are the same orientation, so flip when the pair is more than a
    # quarter turn apart -- otherwise the craft spins the long way round.
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        # Nearly parallel: slerp is numerically unhappy and lerp is exact enough.
        out = q0 + fraction * (q1 - q0)
    else:
        theta = np.arccos(np.clip(dot, -1.0, 1.0))
        sin_theta = np.sin(theta)
        out = (np.sin((1.0 - fraction) * theta) / sin_theta) * q0 + \
              (np.sin(fraction * theta) / sin_theta) * q1

    norm = np.linalg.norm(out)
    return out / norm if norm else q0


class PoseBuffer:
    """Keeps recent snapshots so a viewer can draw between them.

    Args:
        delay (:obj:`float`, optional): How far behind the newest snapshot to
            draw, in snapshot intervals. Slightly over one leaves room for a
            late arrival. Defaults to 1.5.
        history (:obj:`int`, optional): Snapshots to keep. Defaults to 30.
    """

    def __init__(self, delay=1.5, history=30):
        self._delay = float(delay)
        self._history = int(history)
        self._times = []
        self._frames = []

    def __len__(self):
        return len(self._frames)

    @property
    def span(self):
        """:obj:`tuple`: Oldest and newest snapshot times held."""
        if not self._times:
            return (0.0, 0.0)
        return (self._times[0], self._times[-1])

    @property
    def interval(self):
        """:obj:`float`: Mean gap between the snapshots held."""
        if len(self._times) < 2:
            return 0.0
        return (self._times[-1] - self._times[0]) / (len(self._times) - 1)

    def push(self, time, agents):
        """Add a snapshot.

        Out-of-order arrivals are dropped rather than sorted in: by the time one
        shows up the moment it describes has already been drawn, and inserting
        it would make the scene jump backwards.

        Args:
            time (:obj:`float`): Simulation time of the snapshot.
            agents (:obj:`dict`): Agent name to a dict with ``position``,
                ``velocity`` and ``quaternion``.
        """
        time = float(time)
        if self._times and time <= self._times[-1]:
            return

        self._times.append(time)
        self._frames.append({
            name: {
                "position": np.asarray(pose["position"], dtype=np.float64),
                "velocity": np.asarray(pose.get("velocity", [0, 0, 0]), dtype=np.float64),
                "quaternion": np.asarray(pose["quaternion"], dtype=np.float64),
            }
            for name, pose in agents.items()
        })

        while len(self._frames) > self._history:
            self._times.pop(0)
            self._frames.pop(0)

    def render_time(self):
        """The moment a viewer should currently be drawing.

        Returns:
            :obj:`float`: Simulation time, held back by the configured delay.
        """
        if not self._times:
            return 0.0
        return self._times[-1] - self._delay * (self.interval or 0.0)

    def sample(self, time=None):
        """Poses at a given moment, interpolated between real snapshots.

        Before the first snapshot or after the last, the nearest one is held
        rather than extrapolated: guessing forward invents motion that did not
        happen, and a briefly frozen craft reads better than one that slides
        somewhere it never went.

        Args:
            time (:obj:`float`, optional): When to sample. Defaults to
                :meth:`render_time`.

        Returns:
            :obj:`dict`: Agent name to interpolated pose.
        """
        if not self._frames:
            return {}
        if time is None:
            time = self.render_time()

        if time <= self._times[0]:
            return dict(self._frames[0])
        if time >= self._times[-1]:
            return dict(self._frames[-1])

        index = bisect.bisect_right(self._times, time)
        t0, t1 = self._times[index - 1], self._times[index]
        before, after = self._frames[index - 1], self._frames[index]
        fraction = 0.0 if t1 == t0 else (time - t0) / (t1 - t0)

        out = {}
        for name, start in before.items():
            end = after.get(name)
            if end is None:
                # Gone by the later snapshot: hold it rather than fading it
                # through the floor.
                out[name] = start
                continue
            out[name] = {
                "position": start["position"] + fraction * (end["position"] - start["position"]),
                "velocity": start["velocity"] + fraction * (end["velocity"] - start["velocity"]),
                "quaternion": slerp(start["quaternion"], end["quaternion"], fraction),
            }

        # Anything that appeared in the later snapshot shows up on arrival.
        for name, pose in after.items():
            out.setdefault(name, pose)
        return out
