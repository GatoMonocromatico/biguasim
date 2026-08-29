.. _world-server:

===========================
Worlds That Outlive Scripts
===========================

Used as a library, BiguaSim is something a script drives. One Python process
calls :func:`biguasim.make`, owns the engine, ticks it, and exits. The agent
roster, the sensor set and the vehicle dynamics are all fixed by a scenario
dictionary before the first tick, and the simulation exists only for as long as
that one script does.

:mod:`biguasim.server` turns the same simulator into something that runs on its
own and accepts instructions. A **world** is a long-lived process. Clients
connect to it, spawn agents into it, attach sensors mid-flight, fly things, look
at it from wherever they like, and leave -- and the world carries on. Several
clients can do all of that at once without coordinating with each other, or even
knowing about each other.

This section explains how that works and, more importantly, why it is built the
way it is. Most of the design follows from two facts: the simulator is
deterministic, and the engine is the only thing that can decide anything.

.. toctree::
   :maxdepth: 2
   :caption: Topics

   architecture
   running
   actions
   viewing
   recording
   external-dynamics
   design-notes


A note on the word "client"
===========================

This project now uses "client" for two different things, one layer apart, and
confusing them makes nothing make sense. Both are legitimate; they simply belong
to different conversations.

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * -
     - The older meaning
     - The newer meaning
   * - What it is
     - The Python half of BiguaSim
     - A process talking to a running world
   * - Talks to
     - The engine, over shared memory
     - The world, over ZeroMQ
   * - Described in
     - :ref:`develop-sem`
     - This section
   * - Code
     - :class:`biguasim.biguasimclient.BiguaSimClient`
     - :class:`biguasim.client.remote.RemoteWorld`

The older sense is unchanged and still accurate: inside the world process, a
:class:`~biguasim.biguasimclient.BiguaSimClient` still drives the engine through
shared memory and semaphores exactly as it always did.

The new layer sits entirely above that. Where this section says "client" without
qualification, it means a network peer: a pilot, a viewer, a recorder, a control
stack. Where it means the shared-memory half, it says "the engine side".

**Only the world process ever touches shared memory.** That is not a convention,
it is a hard constraint -- the shared-memory channel has one semaphore pair and
one command buffer, so it admits exactly one driver. Network clients never
attach to it, which is precisely why there can be many of them.
