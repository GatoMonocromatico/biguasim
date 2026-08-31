.. _world-running:

===============
Running a World
===============

Serving a World
===============

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


Pacing
======

.. note::

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


Connecting to a World
=====================

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


Steering an Agent
=================

:meth:`~biguasim.client.remote.RemoteWorld.set_control` waits for the world to
acknowledge the command. That is what you want when you are placing a vehicle
and care whether it worked, and it is what the example above uses.

It is also a full round trip, which caps how often a client can steer at
``1/RTT``. For a script nudging a vehicle every few seconds that is invisible.
For a flight controller closing a loop at the world's tick rate it is fatal: at
200 ticks per second on a 5 ms link, waiting for acks alone would hold the loop
to 200 Hz on paper and far less in practice.

:meth:`~biguasim.client.remote.RemoteWorld.stream_control` sends and moves on::

   while flying:
       world.stream_control("uav1", motor_speeds)

Nothing is lost by not waiting. Control is latest-wins under zero-order hold, so
a command that is dropped is superseded rather than missed -- which is also what
a real vehicle does with a missed setpoint. A command the world *rejects* still
surfaces, asynchronously, through
:meth:`~biguasim.client.remote.RemoteWorld.failures`.

The reason this works at all is that the two directions are not symmetric:

===============  ===========================  ===========================
path             shape                        what latency costs
===============  ===========================  ===========================
state            world to client, streamed    a constant offset, no rate
control          client to world, no reply    delay before it takes effect
===============  ===========================  ===========================

State is published every tick and pipelined, so a client reads ticks at the rate
the world produces them no matter how far away it is. Only the control direction
pays for distance, and it pays in *delay* rather than in *rate* -- the vehicle
responds a few ticks later, exactly as it would with a slow ESC.

That distinction is what makes an external flight controller possible at all.
Anything with a serial round-trip dependency has to stay local; anything that
streams can cross a network.


Uncommanded Is Not Unforced
---------------------------

Two different things decide what an agent does, and they are easy to confuse:

**Control abstraction** -- ``cmd_motor_speeds``, ``cmd_vel``, ``accel`` and the
rest -- is what the numbers in a command *mean*. It belongs to the Python
dynamics model, which turns a command into an action.

**Control scheme** is an integer the engine reads to decide what to do with
that action. Every agent starts on 0 and is set to its model's own scheme when
it is created, exactly as :mod:`biguasim.environments` does for the agents in a
scenario.

The consequence is easy to miss: once the scheme is set, **the engine is no
longer integrating the vehicle**. The Python model is, and the engine applies
whatever that model produces. So an agent that is never stepped does not simply
drift -- it has no forces on it at all, gravity included, and hangs in the air.

Which is why every agent is given a neutral command the moment it exists,
rather than being left uncommanded until somebody sends one. Nobody has to send
a command for a vehicle to fall out of the sky, and a world where they did
would be a strange one.

.. note::

   This is only visible once a scheme is set. Before that the engine falls back
   to simulating the actor itself, which looks like ordinary physics right up
   until it stops being -- so the two faults mask each other in both
   directions.


The Build Check
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


Reaching a World From Another Machine
=====================================

The service binds all interfaces by default, so a world is already reachable
from anywhere that can route to the machine serving it. Nothing needs enabling;
what varies is whether a route exists.

.. code-block:: console

   $ python tools/watch_world.py --address 192.168.2.118 --port 8770

Both IPv4 and IPv6 are accepted. ZeroMQ disables IPv6 per socket unless told
otherwise -- an IPv6 address would otherwise be accepted, connected to, and
silently never reach anything -- so it is switched on explicitly, and the
wildcard bind is then dual-stack. ``--ipv4-only`` turns it off.

An IPv6 address may be given plainly. The brackets ZeroMQ needs, because an IPv6
address is as full of colons as the ``host:port`` separator, are added for you:

.. code-block:: console

   $ python tools/watch_world.py --address 2804:60:114:8b00::1 --port 8770

Across the Internet
-------------------

A private IPv4 address -- ``192.168.x.x``, ``10.x.x.x`` -- means nothing outside
its own network, and a home router has no way to know which machine an unsolicited
inbound packet belongs to. Three ways round that, in order of how well they work:

**An overlay network** (Tailscale, ZeroTier) gives every enrolled machine a
stable address that works from anywhere, and handles the traversal itself. It is
also encrypted and authenticated, which matters here; see the warning below.
Nothing in BiguaSim needs configuring -- use the overlay address.

**IPv6**, if the ISP provides it, gives the machine a globally routable address
with no translation in front of it at all. Home routers still firewall inbound
IPv6 by default, but that is a rule to permit rather than an address to forward.

**A public IPv4 address**, from an ISP that will sell one, or a rented machine in
a datacentre. Increasingly ISPs place customers behind carrier-grade NAT, in
which case no amount of router configuration will help and one has to be
requested.

.. danger::

   This protocol has **no encryption and no authentication**. ``client_id`` is
   whatever a client claims it is, and that includes claiming to be an
   administrator listed in ``--admin`` -- so anyone who can reach the port can
   take the world over.

   That is a reasonable design for a trusted or overlay network, where the
   network itself does the authenticating. It is not safe on a public address.
   Do not port-forward or firewall-open a world to the open internet as it
   stands; put it behind an overlay network instead, or add transport security
   first. ZeroMQ's CURVE mechanism is the natural fit.


Watching a World
================

.. code-block:: console

   $ python tools/watch_world.py --port 8770 --fps 60

Opens a local copy of the world in puppet mode and draws what the world reports.
See :ref:`world-viewing` for what that means and what it costs.


Replaying a World
=================

.. code-block:: console

   $ python tools/replay.py run.bslog            # what is in it
   $ python tools/replay.py run.bslog --verify   # re-run it and check

See :ref:`world-recording`.


Checking Reproducibility
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
