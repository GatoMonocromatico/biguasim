.. _world-design-notes:

============
Design Notes
============

Why These Exist
===============

Several things in this code look more complicated than they need to be, and one
looks like a leak that was left in on purpose. Each is a decision with a reason,
and each reason is the kind that gets forgotten and then "fixed" back into a bug.

This page records them.


Shared Memory Has Three Operations
==================================

:class:`biguasim.shmem.Shmem` distinguishes three things that all look like
"finish with this block":

.. list-table::
   :header-rows: 1
   :widths: 16 14 14 56

   * - Operation
     - Keeps file
     - Keeps mapping
     - Used when
   * - ``clear()``
     - yes
     - yes
     - The world resets. Contents are wiped; the engine keeps its view.
   * - ``close()``
     - yes
     - no
     - The block is being reallocated at a new shape.
   * - ``unlink()``
     - no
     - no
     - The sensor is genuinely gone and the engine has let go.

The obvious design has one operation. Here is why it does not work.

The Trap
--------

:meth:`biguasim.environments.BiguaSimEnvironment.reset` calls ``sensor.reset()``
on every sensor. For a long time ``BiguaSimClient.free`` did nothing at all --
it computed a size, called ``seek``, and returned -- so ``reset`` quietly did
nothing, the blocks survived, and ``malloc`` handed the same mapping back
afterwards.

The apparent fix is to make ``free`` actually release. It is a disaster.

The engine holds **its own mapping** of those same files. Removing a file and
recreating it at the same path produces a **new inode**. Python then maps the
new one while the engine goes on writing to the old one, they stop sharing
memory entirely, and every sensor reads zeros forever afterwards.

Nothing about that failure points at shared memory. It looks like the sensors
broke.

So the no-op was load-bearing by accident, and the real fix was to separate the
operation that wipes contents from the one that releases the block.
``tests/test_shmem_lifecycle.py`` pins the inode across ``clear()`` specifically
so this cannot be re-broken.

Release Is Deferred
-------------------

Even genuine release cannot happen immediately. The engine only drops its
mapping when it processes the ``RemoveSensor`` command sitting in that tick's
command buffer.

So :meth:`~biguasim.agents.BiguaSimAgent.remove_sensors` queues the block with
``defer_free``, and ``tick()`` drains the queue after the engine has completed
the tick. Freeing earlier is the inode problem again, in a narrower window.

What Was Actually Leaking
-------------------------

Three independent leaks, none of which a thirty-second script would notice and
all of which are fatal to a process that runs for days:

* ``free()`` released nothing and never dropped its table entry;
* ``Shmem`` kept both the descriptor it opened and the one ``mmap`` dups, so
  every allocation cost two file descriptors;
* ``remove_sensors`` popped its bookkeeping and never released anything at all.

Measured before the fix: 200 add/remove cycles leaked 200 ``/dev/shm`` blocks and
400 descriptors. After: zero of each.

The old ``TODO`` in ``Shmem`` blamed a numpy reference for making the descriptor
impossible to close. It was close but not right -- the obstacle is the **ctypes
buffer**, which exports the mmap and makes ``mmap.close()`` raise ``BufferError``
until it is dropped.


A Killed Agent Keeps Five Blocks
================================

Killing an agent reclaims its sensors. It does **not** reclaim the five small
blocks the agent itself was built with -- action, teleport flag, teleport
command, control scheme, ocean current.

This is the inode problem once more. The engine has no despawn command, so the
actor survives the kill; unlinking buffers it still maps would hand it an
orphan. Leaking five small blocks is strictly better than corrupting the ones
that remain.

The cost is therefore **bounded and predictable**: five blocks per agent ever
killed, and flat over time no matter how long the world runs.
``test_agent_lifecycle_costs_exactly_what_it_should`` asserts that number rather
than pretending it is zero, and asserts that idling adds nothing -- because the
distinction between "bounded" and "growing" is the one that matters, and a test
claiming zero would just fail confusingly.

Adding ``DespawnAgent`` to the plugin C++ removes this entirely.


Bugs Only a Long-Lived World Can Hit
====================================

Several latent bugs surfaced during this work. Most are invisible to a script
that fixes its roster up front, which is why they survived. Two more were
introduced by the fixes for the others, and are recorded here because the pair
masked each other in both directions.

A Sensor With No Rate Went Silent
---------------------------------

``SensorFactory.build_sensor`` copied ``tick_every`` through unchanged. Given
``None``, ``tick_count`` also became ``None``, and ``sensor_data`` returned data
because ``None == None``.

Then ``_tick_sensor`` ran, took the ``else`` branch, and set ``tick_count = 1``.
From that moment ``1 == None`` is false, and **the sensor returned ``None``
forever**.

Scenario sensors always get a rate computed from ``Hz``, so only sensors created
at runtime could hit it -- which nothing did until sensors became something you
attach mid-flight. ``build_sensor`` now defaults a missing rate to 1.

Spawned Agents Were Absent From the State
-----------------------------------------

:class:`~biguasim.environments.BiguaSimEnvironment` chooses its state function
**once, at construction**. A single-agent scenario gets ``_get_single_state``,
which only ever reports the main agent.

In a world whose roster changes this is never right: anything spawned afterwards
simply does not appear, with no error to explain why. Both
:class:`~biguasim.server.world.World` and
:class:`~biguasim.client.viewer.Viewer` force ``_get_full_state``.

``env.step()`` Cannot Express "No Command Yet"
----------------------------------------------

``step`` insists on a command for every model it knows about, and its argument
validation differs by agent count -- a bare list for one, a dict for many.

A freshly spawned agent nobody has commanded yet therefore made it raise, as did
an agent whose dynamics live on the client. The world unrolls the loop instead:
tick the engine, then drive whatever actually has a command. Iteration follows
``_dynamics_dict`` insertion order, which is stable, so this is no less
reproducible than ``step``.


A Spawned Agent Had No Control Scheme
-------------------------------------

:mod:`biguasim.environments` calls ``set_control_scheme`` immediately after its
own ``add_agent``, because ``add_agent`` does not set one and every fresh agent
starts on scheme 0 -- while a quadrotor's dynamics emit scheme 1 and the
underwater models scheme 2. The world's spawn handler did not, so the engine
read thrust as something else entirely.

Nothing raised. The roster was right, the controls dictionary held the right
numbers, and the vehicle sat where it landed and answered nothing. No agent
spawned into a running world had ever responded to a command.

The test that missed it asserted that the controls dictionary received the
numbers. The one that replaced it asserts the vehicle climbs.

Uncommanded Was Not Unforced
----------------------------

Fixing the scheme broke gravity, which is the other half of the same fact.

Once an agent has a control scheme the engine stops integrating it: the Python
dynamics model does, and the engine applies what that model produces. ``_advance``
skipped any agent whose command was ``None``, so an agent nobody had commanded
was never stepped -- and an agent that is never stepped has no forces on it at
all, gravity included. Both spawned and scenario-loaded agents hung in mid-air.

Before the scheme was set this could not happen, because the engine fell back to
simulating the actor itself. So the two faults masked each other in both
directions: no scheme looked like working physics, and fixing the scheme broke
physics for anything not being flown. Every agent now gets a neutral command of
the right width the moment it exists.

An Actor Created Inside Geometry Never Reports
-----------------------------------------------

A vehicle created intersecting the ground never initialises its physics body,
and then reports zeros for the rest of the run. Teleports still arrive and the
engine still logs them; the actor is simply inert.

This cost a long detour. The viewer created every puppet at the origin, so a
HolybroX500 -- which rests at 0.216 m and fails below it -- read zeros forever,
and the obvious conclusion was that the plugin had no teleport handler for that
vehicle. It has one. The engine log showed the actor moving the whole time while
the readback said otherwise, and the probe used to investigate had the same
fault as the code under investigation, so it agreed.

Puppets are now created at the pose the world reported, lifted clear. A landed
vehicle's own position is not a safe spawn point: it is exactly on the boundary.

A Reply The World Could Not Serialize Stopped It
-------------------------------------------------

Provisioning a flight controller returned the record it stores, which holds the
``Popen`` handles for the child processes, and that record went into the reply.
msgpack refuses a ``Popen``.

The refusal did not arrive where the mistake was. ``proto.pack`` sat outside the
``try`` in ``_drain_requests``, so the exception escaped the request loop,
escaped the tick loop, and stopped the world for every connected client over one
client's request -- the exact failure the surrounding guard exists to prevent.
It covered building a reply, not packing one.

The lesson generalises past the immediate bug: a guard around handling is not a
guard around answering.

Smaller Decisions
=================

**The command buffer is written in one copy.** It used to be a per-byte Python
loop. Irrelevant for a script issuing occasional commands; measurable for a
world writing them every tick.

**Backpressure exists but almost never triggers.** ``max_buffer`` is 1 GiB, so
overflow is close to unreachable. The accounting is there anyway because the
failure mode -- a burst from several clients overrunning the buffer -- is silent
data loss, and the deferral path is a dozen lines.

**Subscribers have a bounded receive queue.** A client slower than the world
should lose old messages rather than accumulate a backlog it then reads as
though it were the present. Recorders, which want everything, raise it.

**Sensor topics are published unconditionally and filtered by ZeroMQ.** The
world does not track who is subscribed to what. This keeps subscription state
out of the tick loop entirely, at the cost of serializing a sensor nobody is
watching. Worth revisiting when there is a real camera load; not before.

**Out-of-order snapshots are dropped by the viewer, not sorted in.** By the time
a late snapshot arrives, the moment it describes has already been drawn.
Inserting it would make the scene jump backwards, which is worse than the gap it
was trying to fill.


Things Worth Checking That Were Not
===================================

**Water and wave determinism on viewers.** Weather is entirely command-driven, so
replaying world-mutating actions to a viewer's local environment syncs it for
free. The water surface is engine-side and was never exercised. If it is not a
pure function of simulation tick, a surface vehicle will render sunk or floating
on a viewer while the world thinks it is fine.

**Cross-machine determinism.** Everything measured here was one machine, one
GPU, one binary. Different hardware is an open question, and the device pinning
in recordings is a partial answer at best.

**Load.** The largest test was three agents. Nothing here has met fifty.
