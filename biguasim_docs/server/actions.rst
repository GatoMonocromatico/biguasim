.. _world-actions:

=======
Actions
=======

Everything a client can ask a world to do is an action. There is no other verb.

That uniformity is what makes the rest possible: because every request is a
value with a tick and an identity on it, requests can be sorted, logged,
replayed and refused by one mechanism rather than several.


Two kinds, and why the distinction matters
==========================================

.. list-table::
   :header-rows: 1
   :widths: 22 39 39

   * -
     - World-mutating
     - Control
   * - Examples
     - ``SpawnAgent``, ``KillAgent``, ``AddSensor``, ``RemoveSensor``,
       ``RotateSensor``, ``SetWeather``, ``SetDayTime``, ``SetFogDensity``,
       ``SetControlDefaults``
     - ``SetControl``, ``SetPose``
   * - Losing one
     - Changes the world permanently. Unacceptable.
     - Harmless.
   * - Why
     - There is no later message that restores a sensor you failed to attach.
     - The previous command stays in force, which is what a real vehicle does
       between messages anyway.
   * - Consumes engine command buffer
     - Yes, so it can be deferred under load
     - No

Control actions are a **zero-order hold**: whatever an agent was last told
remains in force until something supersedes it. This is not a simplification, it
is how vehicles actually behave -- an autopilot does not stop holding an
attitude because no new message arrived this millisecond.

Because of that, dropping a control message costs almost nothing, and the design
never has to work hard to guarantee their delivery.


Ordering and identity
=====================

Every action carries three fields beyond its payload:

``target_tick``
   The tick it is meant for. Clients get this from
   :attr:`~biguasim.server.world.World.next_tick`.

``client_id``
   Who sent it. Also the ownership key. The world overwrites whatever a client
   claims here with the identity of the connection it arrived on -- otherwise
   one client could act as another simply by saying so.

``seq``
   A per-client counter. Breaks ties between two actions the same client sent
   for the same tick.

Together these form
:attr:`~biguasim.server.actions.Action.order_key`, and that tuple is the **only**
ordering the world is permitted to use. See :ref:`world-architecture`.


Ownership
=========

A client that spawns an agent owns it. Only the owner may control, kill or
attach sensors to it; anyone else is refused at submit time.

Agents that came from the world's starting scenario are owned by nobody, and
anyone may act on them. That is deliberate: they are the world's furniture, not
anyone's property, and a shared world usually wants some of it drivable by
whoever shows up.

Clients listed as admin (``--admin``) bypass ownership entirely.


Sensor names are assigned, not requested
========================================

:class:`~biguasim.server.actions.AddSensor` takes an optional ``sensor_name``,
and the world will happily ignore it.

The shared-memory key backing a sensor is derived from the agent name plus the
sensor name, on both the Python and engine sides. Two clients each attaching a
sensor called ``camera`` to the same agent would therefore be pointed at the
same block, and would silently overwrite each other's data.

Since that key format cannot change without engine work, the world assigns
unique names instead -- ``<type>_<client>_<n>`` -- which keeps the existing
derivation intact and makes collisions impossible rather than unlikely. The name
the world actually used appears in the published state.


Sensor rates
============

``AddSensor`` takes an optional ``hz``, defaulting to the world's tick rate.
Sampling is implemented as a tick divider rather than a real clock, so the rate
must divide the tick rate evenly; anything else is refused rather than silently
rounded, because a sensor quietly running at 6.67 Hz when you asked for 7 is the
kind of thing that is discovered months later in a plot.


Killing an agent
================

The engine has no despawn command. ``KillAgent`` is therefore a **soft kill**:
sensors removed, dynamics stopped, dropped from the roster and the broadcast,
and parked far out of range.

The actor survives, and so do the five small shared-memory blocks it was built
with. They cannot be released while the engine still maps them -- see
:ref:`world-design-notes` for why that would be worse than leaking them.

The cost is therefore bounded and permanent: five blocks per agent ever killed,
none of which grows over time. Hard despawn needs a ``DespawnAgent`` handler in
the plugin C++, which does not exist yet.


Reference
=========

Every action type, with its fields: :doc:`../biguasim/server-api`.
