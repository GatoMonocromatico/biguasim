.. _world-architecture:

============
Architecture
============

The Problem
===========

Several people want to interact with one simulation at the same time, each with
their own agents, each looking at it from wherever they like -- and the
simulation should behave like a place that exists rather than a function
somebody called.

That sounds like a multiplayer game problem, and games have three well-worn
answers. It is worth knowing why only part of one of them applies here.

**Deterministic lockstep** (StarCraft, Age of Empires) sends only inputs and
requires every participant to run an identical simulation, advancing in turns
nobody may skip. **Authoritative server with client prediction** (the Quake
lineage, and most shooters since) keeps one true world on a server that never
waits, and hides latency on each client by simulating its own input immediately
and reconciling when the server disagrees. **Rollback** (fighting games) is
lockstep plus speculative execution, and is excellent for two players.

BiguaSim already *is* the authoritative half. The engine plus the dynamics loop
is the single source of truth, and always was. What was missing was never a
netcode model; it was a front door -- something that lets N processes each
supply commands for their own agents and receive only what they are entitled to
see.

And the expensive half of the shooter answer turns out to be unnecessary here.

.. admonition:: Why there is no client-side prediction

   Prediction and reconciliation exist for exactly one reason: hiding latency
   from a human hand. They are the hardest part of game networking, and they are
   not needed here.

   An autonomous agent does not perceive latency. It perceives a delayed sensor
   reading, which is a control problem it should already be solving, because a
   real vehicle has exactly the same delay.

   Even for a human flying manually, a quadrotor has more inertia than a network
   has latency. It takes 100-200 ms of real physics for a stick input to visibly
   change attitude; a 30 ms round trip disappears inside that. A shooter needs
   prediction because an avatar accelerates instantly. A drone does not.

   So the world gets the useful half of the shooter architecture -- one
   authoritative simulation, snapshots to everyone, nobody waiting -- and skips
   the part that would have been most of the work. If a real pilot ever complains,
   that is the moment to build it, and not before.


The Shape of It
===============

::

    pilot ─┐
    viewer ─┼── ZeroMQ ──→  World process  ──shared memory──→  UE5 engine
    recorder ─┘              (actions in,
                              state out)

Everything a client can ask for is an **action**. Actions go onto a queue, the
queue is drained at a tick boundary, and each action is applied in a defined
order. This is not new machinery: the engine-side ``CommandCenter`` has always
worked this way. The world simply promotes that pattern into a first-class,
network-facing API and puts rules around it.

Each tick, in order:

1. Drain the request socket; queue whatever arrived for a tick slightly ahead.
2. Take everything now due, **sorted**, and apply it.
3. Advance the engine one tick, then drive whatever has a command.
4. Publish state, and any sensor topic somebody subscribed to.
5. Return failures to whoever caused them.


Two Rules Carry the Design
==========================

Everything else is detail. These two are load-bearing.

Actions Apply in ``(target_tick, client_id, seq)`` Order
--------------------------------------------------------

When two clients' messages cross on the wire, the order they *arrive* in is a
property of the network that day. It will not be the same twice, and it is not
recorded anywhere.

If arrival order were allowed to reach the world, then two runs of the same
session would differ, and a recording could never reproduce the run it came
from. Sorting by a key made of things that *are* recorded -- the tick an action
was aimed at, who sent it, and their own counter -- costs one sort per tick and
buys reproducibility outright.

This is why :meth:`~biguasim.server.world.World.submit` is separate from
applying: submission is when an action arrives, and the world deliberately
forgets that fact immediately.

The Recorded Tick Is the One an Action Ran On
---------------------------------------------

Clients aim actions a fixed few ticks ahead (see `Input delay`_ below). Usually
an action lands exactly there. Under load it can slip -- the engine's command
buffer fills, or an action arrives for a tick that has already run.

A log that recorded intentions would then describe a run that never happened,
and replaying it would produce a different one. So the world records where each
action actually landed, and :mod:`~biguasim.server.recording` writes that.


Input Delay
===========

Clients submit actions for ``current_tick + D``, where ``D`` is a small constant
(:data:`~biguasim.server.world.DEFAULT_INPUT_DELAY`, three ticks). The world
advertises it in the connection greeting, and
:attr:`~biguasim.server.world.World.next_tick` is the tick a client should aim at.

The delay exists so that an action has time to cross the network before the tick
it belongs to. This is the same trick real-time strategy games use, for the same
reason.

.. warning::

   ``D`` must be a **fixed constant in the protocol**, never adapted to measured
   round-trip time.

   An adaptive delay makes the same recording replay differently on a different
   network, because the tick each action lands on would depend on the connection
   rather than on the recording. That silently destroys reproducibility, which is
   the property everything else here is built on.

An action aimed at a tick that has already run cannot be honoured as asked. It
is moved to the next available tick rather than dropped -- the client's intent
survives, and the log records where it really landed.


What the World Is Authoritative About
=====================================

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Decided by
     - What
   * - The world
     - Collision, contact, sensor simulation, the official position of every
       agent, who owns what, and what tick it is.
   * - The owning client
     - What its agents are trying to do. And, for an agent spawned
       ``externally_driven``, where that agent is -- see :ref:`external-dynamics`.
   * - Each viewer alone
     - Where its camera is pointing, and how to smooth between snapshots.

That last row is the one that makes watching cheap. See :ref:`world-viewing`.


Rendering Is Not Sensor Simulation
==================================

These get conflated, and separating them is what makes the cost model work.

**Rendering a view** is drawing the world from a pose. A client does this in its
own engine with its own copy of the world, so it costs that viewer's GPU and
nothing else. Ten people watching cost the world the same as one.

**Sensor simulation** is not a render. An imaging sonar is a raytrace against
the octree; a rangefinder is a set of casts; collision is a query. These produce
data that must be authoritative, so the world computes them.

Consequently the world's GPU load tracks **the number of active sensors, not the
number of viewers**. A viewer that only wants to look around costs a few
kilobytes a tick. A viewer that wants a photoreal camera feed asks for a camera
sensor, and that one costs a render -- correctly, and visibly.


Errors Happen at Two Different Times
====================================

Not a quirk; the two kinds are knowable at different moments, so pretending
otherwise would mean either lying or waiting.

**Whether a client may act** is knowable the instant it asks. Ownership is
already known, so an unauthorized action is refused synchronously, and
:meth:`~biguasim.client.remote.RemoteWorld.submit` raises.

**Whether the action works** is not knowable until it runs, several ticks later.
By then the request has long since been answered. So execution failures come
back asynchronously, through
:meth:`~biguasim.client.remote.RemoteWorld.failures`.

Crucially, an execution failure is contained. A client asking to drive an agent
that was killed a moment ago is ordinary traffic in a shared world, not grounds
for stopping it. The failure goes to that client and the world carries on for
everyone else.


There Is No Reset
=================

:meth:`biguasim.environments.BiguaSimEnvironment.reset` is not reachable through
the world. A living world has spawn and kill; a global reset would silently pull
the floor out from under every other client connected to it.

This matters more than it sounds, because ``reset`` is also the operation that
tears down and rebuilds the entire agent roster. In a shared world that is not a
reset, it is an outage.
