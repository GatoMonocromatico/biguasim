.. _world-recording:

=======================
Recording and Replaying
=======================

The measurement everything rests on
===================================

Before any of this was designed, one question had to be answered: does the same
input produce the same run?

It does. Every trial was **bit-identical**, comparing separate processes and
separate engine boots:

.. list-table::
   :header-rows: 1
   :widths: 60 40

   * - Probe
     - Result
   * - torch dynamics on cpu, same process and across processes
     - bit-identical
   * - torch dynamics on cuda, same process and across processes
     - bit-identical
   * - engine, one agent, open air, 200 ticks
     - bit-identical
   * - engine, contacts (56 collision samples) and raycasts, 300 ticks
     - bit-identical
   * - engine, three agents, 200 ticks (actor ordering)
     - bit-identical
   * - a scripted action stream through the world, with a mid-run spawn,
       a mid-run sensor and a kill
     - bit-identical

This was not expected. GPU reductions are usually where reproducibility dies.
The reason it survives here is that the vehicle models carry ``.double()``
throughout (:mod:`biguasim.dynamics.base_model`) -- float64 arithmetic on this
workload is stable in a way float32 would not have been.

Reproduce it with ``tools/determinism_probe.py``; it is guarded by
``tests/test_determinism.py``.


Why that changes what a recording is
====================================

There are two kinds of replay, and they are not interchangeable.

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * -
     - Recorded playback (a rosbag)
     - Re-execution (what this does)
   * - Stores
     - What happened
     - What was asked
   * - Size
     - Megabytes upward
     - ~50 bytes a tick
   * - Needs determinism
     - No
     - Yes
   * - Can be edited and re-run
     - **No**
     - **Yes**
   * - Is
     - A movie
     - A simulation

Because the simulator is reproducible, a recording does not have to store the
run. A 200-tick session with 48 actions is about 10 KB.

The size is the smaller half of it. Storing asks rather than results means a
recording can be **edited and re-run**: change one action and you find out what
would have happened instead. A recording of state can only be watched.

That property is easy to lose by accident, so there is a test that edits an
action and insists the outcome changes
(``test_replay_re_executes_rather_than_playing_back``).


Keyframes
=========

Full state is written every so often anyway, despite replay being exact.

Two reasons, neither of them correctness. Seeking: playback can jump to minute
forty without simulating the first thirty-nine. And detection: a replay that has
drifted -- because something changed underneath it -- says so at the next
keyframe rather than diverging quietly and being believed.


The device caveat
=================

.. warning::

   cpu and cuda results are **not** bit-identical. They agree only to float64
   rounding, around 2e-17.

   :mod:`biguasim.environments` selects cuda whenever it is available, and
   :func:`biguasim.util.gpu` chooses between GPUs by free VRAM -- so on a
   multi-GPU host **the device can change between runs on its own**, with
   nothing in the run to say it did.

The recording pins the device in its header, and replaying on a different one is
refused rather than warned about.

Refused, because the drift is small. A warning gets ignored, and a 2e-17
divergence that grows over a few thousand ticks produces a run that is subtly
wrong and entirely plausible -- much worse than one that fails outright. Pass
``--any-device`` if you genuinely want it.


Failed actions are recorded too
===============================

If an action failed during the original run, it is kept in the log, error and
all. A faithful replay should fail the same way at the same tick. Dropping
failures would produce a replay of a tidier session than the one that happened.


Using it
========

.. code-block:: python

   from biguasim.server import Recorder, World

   with Recorder("run.bslog", scenario, keyframe_every=200) as rec:
       with World(scenario, record=rec.record_action) as world:
           for _ in range(1000):
               rec.observe(world.tick, world.step())

Or, when serving, hand a recorder to
:class:`~biguasim.server.service.WorldService` and it wires both halves itself.

.. code-block:: console

   $ python tools/replay.py run.bslog --verify
   run.bslog
     build      SkyDive:Pier-Harbor:494563d30ca0
     device     cuda:0
     actions    48
     keyframes  8
     by kind    add_sensor x1, kill_agent x1, set_control x45, spawn_agent x1
     failed     3 (these should fail again on replay)

   replaying on cuda:0...
     compared 8 keyframes, matched 8

   reproduced exactly


Format
======

Length-prefixed msgpack: a header, then records in the order they happened.

Deliberately dull and dependency-free. MCAP would give Foxglove tooling and
seeking for free and is the obvious next step if the recordings ever need to be
read by anything else -- but that is a transcoding job against a format this
simple, not a redesign, so it was not worth taking the dependency up front.


.. seealso::

   :doc:`../biguasim/server-api` for :class:`~biguasim.server.recording.Recorder`,
   :class:`~biguasim.server.recording.Recording` and
   :func:`~biguasim.server.recording.replay`.
