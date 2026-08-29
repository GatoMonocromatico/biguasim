.. _world-running:

===============
Running a World
===============

Serving one
===========

.. code-block:: console

   $ python tools/serve_world.py --package SkyDive --world Pier-Harbor \
         --port 8770 --rate 20 --agent uav0:DjiMatrice

Requests are taken on ``--port``; state is published on ``--port + 1``. Two
sockets, because they are different conversations: requests are point-to-point
and need answers, published state is one-to-many and needs none.

A world may start with no agents at all. ``--agent`` is a convenience for the
common case where something should already be flying; normally agents are
spawned by whoever wants them, and belong to that client.

Useful options:

``--rate``
   Ticks per simulated second. This is the world's clock.

``--fps``
   Wall-clock cap. Defaults to ``--rate``. Pass ``0`` to let it run flat out.

``--input-delay``
   Ticks between a client submitting and the action landing. See
   :ref:`world-architecture`.

``--admin``
   A client id exempt from ownership checks. Repeatable.

``--viewport``
   Show the engine window on the serving machine. Off by default -- a world is
   usually headless and watched from elsewhere.


Pacing: why the default is real time
====================================

.. important::

   Left free-running, the engine ticks as fast as it can, which is far faster
   than real time. In testing a world reached tick 175 within one second of
   starting.

   Every viewer is then permanently behind, reading a backlog rather than the
   present, and no amount of client-side cleverness fixes it -- the world is
   simply generating history faster than anyone can consume it.

So ``--fps`` defaults to ``--rate``, and a shared world runs at wall-clock speed.

Batch work wants the opposite, and should pass ``--fps 0``. A reinforcement
learning run or a dataset generation sweep has no viewers to fall behind, and
every tick it can get is a tick earned.

This is deliberately the scenario's existing ``frames_per_sec`` knob rather than
new machinery. The engine already knows how to pace itself; the world had no
business reinventing that.


Connecting to one
=================

.. code-block:: python

   from biguasim.client import RemoteWorld

   scenario = {"package_name": "SkyDive", "world": "Pier-Harbor"}

   with RemoteWorld(port=8770, client_id="pilot", scenario_cfg=scenario) as world:
       world.watch_state()

       landed = world.spawn_agent(
           "uav1", "DjiMatrice",
           location=(10.0, 0.0, 25.0),
           sensors=[{"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
                     "configuration": {"UseCOM": True, "UseRPY": False}}])

       world.set_control("uav1", [330.0] * 4)
       state = world.wait_for_tick(landed + 20)
       print(state["agents"]["uav1"]["position"])

``spawn_agent`` returns the tick the spawn is scheduled for, not the tick it is
called on. Nothing a client asks for happens immediately, which is why
:meth:`~biguasim.client.remote.RemoteWorld.wait_for_tick` exists -- it reads
published state until the world has actually got there.

Leaving the ``with`` block says goodbye properly. That matters; see
`Disconnection`_.


The build check
===============

A connecting client must be running the same world build, or it is refused.

This looks unfriendly and is not. A client draws the world with **its own copy**
of the package. If that copy differs from the world's, its collision geometry
disagrees with the authoritative one: things rest slightly inside surfaces,
raycasts hit different objects, and a vehicle appears to clip through a pier
that is really there.

Nothing about that failure looks like a version problem when you are staring at
it. It looks like the physics is broken. Refusing the connection turns a long
confusing afternoon into one clear error message.

The identity is a digest of the package name, the world name and the installed
world configuration -- see :func:`biguasim.server.protocol.build_id`.


Disconnection
=============

When a client says goodbye, or is dropped, the world has to decide what happens
to the agents it owned. Leaving them on their last command is not an option: a
quadrotor whose pilot closed their laptop would keep climbing at full power
until it left the map.

So each agent falls back to whatever its owner registered with
:meth:`~biguasim.client.remote.RemoteWorld.set_control_defaults`, and is
soft-killed if no default was set.

.. code-block:: python

   world.spawn_agent("uav1", "DjiMatrice", ...)
   world.set_control_defaults("uav1", [0.0] * 4)   # cut power if I vanish

Registering a default is the difference between an agent that survives your
disconnection and one that does not. Which you want depends on the agent, so the
world does not guess.


Watching one
============

.. code-block:: console

   $ python tools/watch_world.py --port 8770 --fps 60

Opens a local copy of the world in puppet mode and draws what the world reports.
See :ref:`world-viewing` for what that means and what it costs.


Replaying one
=============

.. code-block:: console

   $ python tools/replay.py run.bslog            # what is in it
   $ python tools/replay.py run.bslog --verify   # re-run it and check

See :ref:`world-recording`.


Checking reproducibility
========================

.. code-block:: console

   $ python tools/determinism_probe.py --scenario contact --out a.npz
   $ python tools/determinism_probe.py --scenario contact --out b.npz
   $ python tools/determinism_probe.py --compare a.npz b.npz

Four scenarios, in increasing order of how likely they are to break: ``open``
(one agent, no contacts), ``contact`` (flown into terrain, plus raycasts),
``multi`` (three agents, exercising engine actor ordering) and ``world`` (a
scripted action stream through :class:`~biguasim.server.world.World`, including
a mid-run spawn and a kill).

Worth re-running after any change to dynamics, sensors or the tick loop.
Reproducibility is the assumption underneath recording and replay, and it is the
kind of property that breaks quietly.
