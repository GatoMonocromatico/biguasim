.. _world-viewing:

========
Watching
========

Several people watching one simulation, each from their own angle, is the thing
this was built for. It is also, happily, the cheap part.


Poses over the wire, pixels at home
===================================

A viewer opens **its own full copy of the world** and puts it in puppet mode: it
simulates nothing, and every frame it moves each vehicle to where the world says
it is and draws the result.

The alternative would be rendering on the server and streaming video. That is a
real technique -- Unreal supports it -- but it costs the server a render and an
encode *per viewer*, which puts a hard ceiling on how many people can watch.

Sending poses instead means:

* the wire carries a few kilobytes a tick, published **once** however many
  people are looking, because ZeroMQ fans it out;
* the pixels are made on the machine that wants them, in the same engine, with
  the same assets, at full fidelity -- it is not a simplified view;
* ten people watching cost the world exactly what one does.

The price is that every viewer needs the world package installed, and needs it
to match. That is what the build check at connection time is for; see
:ref:`world-running`.


The camera is nobody else's business
====================================

Where a viewer points its camera never leaves that machine. The world is not
told, does not care, and does no work for it.

This is worth stating plainly because it is the whole answer to "can lots of
people look at the same simulation from different angles" -- yes, and it costs
nothing, because looking is not an operation the world participates in.

A viewer that wants something the world genuinely must compute -- a photoreal
camera feed from an arbitrary pose, a sonar return -- asks for a **sensor**
instead. That does cost a render, and it should. The two cases are different and
the design keeps them visibly different:

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * -
     - View camera
     - Sensor camera
   * - Lives
     - In the viewer only
     - In the world
   * - World knows about it
     - No
     - Yes
   * - Costs the world
     - Nothing
     - One render per tick
   * - Produces data
     - No
     - Yes, and it is recorded
   * - Can feed an agent
     - No
     - Yes


Drawing between snapshots
=========================

State arrives at the world's tick rate -- twenty a second, say. Viewers draw
faster than that. Something has to fill the gap or the scene visibly steps.

The answer is the one every networked game uses: **draw the recent past**. Held
back a little over one snapshot interval, there is almost always real state on
both sides of the moment being drawn, so the answer is an interpolation between
two things that actually happened rather than a guess about what happens next.

The cost is a fixed sliver of latency, around 75 ms at 20 Hz. The benefit is
that a late or dropped snapshot passes completely unnoticed, because the viewer
was not relying on it having arrived yet.

Two details that are easy to get wrong:

**Outside the held range, poses are held rather than extrapolated.**
Extrapolating invents motion that did not happen. A craft that freezes for a
moment reads as a hiccup; one that slides somewhere it never went reads as
broken physics, and worse, it lies.

**Rotations interpolate along the shorter arc.** A quaternion and its negation
describe the same orientation, so a naive interpolation between a pair that
happen to have opposite signs sends the vehicle spinning the long way round.
:func:`~biguasim.client.interpolation.slerp` flips the sign when the dot product
is negative.


Smoothing never touches data
============================

.. important::

   Interpolation exists only in the drawing path. Sensor data is never smoothed,
   predicted, or interpolated.

   This is structural rather than a setting. There is no flag to turn it off,
   because a flag is something that can be left in the wrong position by someone
   who then publishes the results.

   A snapshot arrives and forks: one branch goes to the renderer and may be
   smoothed however looks best, the other goes to the sensor path raw and
   tick-stamped. They never rejoin. An interpolated pose is a plausible fiction,
   and it has no business anywhere near data anyone might use for anything.


Agents appearing and disappearing
=================================

Puppets are created as the roster is discovered, so a viewer assumes nothing
about who is in the world when it opens, and picks up agents spawned later
automatically.

The vehicle type is published alongside the pose -- a viewer cannot draw a
vehicle without knowing which one it is -- and is tracked outside the
interpolation buffer, which only carries quantities that can meaningfully be
blended between two snapshots. A type cannot.

When an agent leaves, its puppet is parked in the same corner the world uses,
derived from the same bounds -- the local engine has no despawn either, for the
same reason and at the same cost as the world side.

The park is re-issued every frame until the puppet's own dynamics sensor reports
that it arrived, and complains if it never does. That is not belt and braces: a
teleport the engine declines to honour is not an error, and the one-shot version
of this left a vehicle drawn in a world that no longer contained it, with nothing
to correct it -- even though the roster is a full snapshot rather than a delta,
so the world went on saying the agent was gone twenty times a second.


.. seealso::

   :doc:`../biguasim/client-api` for :class:`~biguasim.client.viewer.Viewer` and
   :class:`~biguasim.client.interpolation.PoseBuffer`.


Puppets that never appear
=========================

A puppet is created locally and then teleported to the pose the world published,
every frame. It can still fail to appear, and the reason is worth knowing
because nothing about it looks like what it is.

**An actor created inside geometry never initialises its physics body.** After
that it reports zeros forever, whatever is sent to it. The teleports are
delivered and the engine logs them; the actor is simply inert. Measured on
``CompetionMap`` with a landed HolybroX500 at ``[3, 2, 0.216]``:

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Created at
     - Reports
     - Drawn?
   * - ``(0, 0, 0)``
     - ``[0, 0, 0]``
     - no
   * - ``(3, 2, 0.216)`` -- its own pose
     - ``[0, 0, 0]``
     - no
   * - ``(3, 2, 2.216)`` -- lifted
     - ``[3, 2, 0.216]``
     - yes

Note the middle row: a landed vehicle's *own* pose is not a safe spawn point,
because it is exactly on the boundary. So puppets are created at the pose the
world reported plus :data:`~biguasim.client.viewer.PUPPET_SPAWN_LIFT`, and the
first teleport brings them down to the right place immediately.

The viewer also watches for the failure. When a live puppet stays more than
:data:`~biguasim.client.viewer.TRACKING_TOLERANCE` metres from where it was put
for :data:`~biguasim.client.viewer.TRACKING_ATTEMPTS` consecutive frames, it
says so once, and says whether the actor is reporting all zeros -- which is the
signature of this particular fault rather than of a teleport being refused.

.. note::

   Puppets are moved with ``teleport()``, not ``set_physics_state()``. The
   latter leaves the actor simulating, so it falls back toward wherever physics
   wants it between frames: asked for ``[6, 6, 4]``, a DjiMatrice settles on
   ``[6, 6, 3.99]`` under ``teleport`` and on ``[1.51, 1.51, 1.18]`` under
   ``set_physics_state``. That was the source of the drift viewers used to
   show. Velocity is dropped along with it and is no loss -- nothing here
   integrates, and the pose is replaced wholesale on the next frame.
