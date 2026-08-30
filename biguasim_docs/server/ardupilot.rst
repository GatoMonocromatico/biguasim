.. _ardupilot-pilot:

Flying ArduPilot in a served world
==================================

A world can hold vehicles it integrates itself, driven by
:meth:`~biguasim.client.remote.RemoteWorld.set_control`. This page is about the
other kind: a vehicle flown by a real ArduPilot, with its EKF, its flight
modes, and a GCS you can point at it.

The piece that makes it work is a **pilot** -- one process per vehicle, holding
a SITL, an FDM bridge and a connection to the world. It is
:class:`~biguasim.ardubridge.RemoteArduRunner`, driven either by
``tools/ardu_pilot.py`` with no ROS at all, or by the ROS bridge node, which is
the same runner with publishers attached.


Where the pilot runs, and why it is not a preference
----------------------------------------------------

**On the machine running the world.**

ArduPilot's JSON backend is a lockstep protocol. ``JSON::update()`` is
``output_servos()`` followed by a blocking ``recv_fdm()`` -- one frame in
flight -- and the servos for frame N+1 are computed from the state that came
back for frame N. That is a control dependency, not a protocol choice, so
there is nothing to pipeline. Put a network in the middle and the entire flight
loop is capped at ``1/RTT``: at 45 ms that is 22 Hz, and a multirotor does not
fly at 22 Hz.

Co-located, the same loop runs over loopback at roughly 0.05 ms and the
question stops existing. What crosses the network instead is **MAVLink**, which
was designed for telemetry radios and is entirely comfortable with 45 ms.

This is the split the real airframe already uses:

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Real drone
     - Served world
     - Tolerates delay?
   * - Pixhawk to airframe
     - SITL to world
     - No -- lockstep
   * - Jetson to Pixhawk
     - autonomy to SITL
     - Somewhat
   * - Ground station to drone
     - your laptop to the server
     - Yes, by design

The rule that falls out is worth remembering on its own: **anything with a
serial round-trip dependency stays beside the world; anything that streams, or
was built for a lossy link, can cross the network.**


The vehicle exists only while its pilot does
--------------------------------------------

:meth:`~biguasim.ardubridge.RemoteArduRunner.start` binds the FDM socket, waits
for SITL's opening servo packet, and *only then* spawns the agent.

That ordering is load-bearing, and it exists because of a difference between
the two bridges that is easy to miss. The local bridge cannot simulate without
ArduPilot at all -- ``env.step()`` is only called when a servo frame arrives.
A served world **free-runs on its own loop**. An agent spawned into it before
its flight controller is up is not politely paused; it is integrated with
whatever :class:`~biguasim.server.actions.SetControl` last said, which for a
quadrotor means falling out of the sky while SITL boots.

Spawning on connect removes that window entirely. ArduPilot spends the spawn
blocked in ``recv_fdm``, which costs nothing: ``time_now_us`` only advances on
a received frame, so the wait is invisible to its own clock.

:meth:`~biguasim.ardubridge.RemoteArduRunner.close` is the symmetric half and
retires the agent. For the case where it never runs -- a pilot that crashes --
:meth:`start` also registers zero-throttle defaults, so the vehicle outlives
its pilot on a safe standing order rather than on whatever throttle it last
had. See :ref:`disconnection <world-running>`.

.. warning::

   **An agent name is never reusable.** ``kill_agent`` is a soft kill: the
   world stops simulating the agent and drops it from the roster, but nothing
   in the UE5 plugin destroys an actor, so the engine keeps the name forever.
   A restarted pilot needs a new ``--agent``. Until a ``DespawnAgent`` command
   exists in the plugin this cannot be fixed on the Python side, and the runner
   settles for explaining the engine's otherwise-baffling
   ``Duplicate agent name``.


Ports
-----

ArduPilot adds ``10 * instance`` to every port it uses
(``SITL_cmdline.cpp:426-441``), so a single instance number is the whole of
multi-vehicle port allocation.

.. list-table::
   :header-rows: 1
   :widths: 30 15 25 30

   * - Purpose
     - Base
     - Instance N
     - Owner
   * - MAVLink TCP
     - 5760
     - ``5760 + 10N``
     - SITL
   * - RC in
     - 5501
     - ``5501 + 10N``
     - SITL
   * - FDM servo out
     - 9002
     - ``9002 + 10N``
     - the pilot's bridge
   * - FDM state in
     - 9003
     - ``9003 + 10N``
     - SITL
   * - World requests
     - 8770
     - 8770
     - the world -- shared
   * - World state
     - 8771
     - 8771
     - the world -- shared


Flying one, with no ROS involved
--------------------------------

Three terminals on the machine running the world. First the world::

   python tools/serve_world.py --package Competition --world CompetionMap --port 8770

Then the pilot, which prints the ports it wants and waits::

   python tools/ardu_pilot.py --package Competition --world CompetionMap \
       --vehicle HolybroX500 --agent uav0 --location 0,0,1

Then SITL, pointed at the port the pilot named::

   sim_vehicle.py -v ArduCopter -f json:127.0.0.1 -I 0 -N

The pilot spawns ``uav0`` the moment SITL says hello. Point a GCS at
``tcp:127.0.0.1:5760``, or forward it to another machine::

   mavproxy.py --master tcp:127.0.0.1:5760 --out tcpin:0.0.0.0:14550

``tcpin`` makes MAVProxy *listen* rather than dial, so the server never needs
to know where you are -- which also means it keeps working when your address
changes.

Watch it from anywhere with ``tools/watch_world.py``, which needs nothing from
the pilot: the vehicle is in the world like any other.


A second vehicle
----------------

A second pilot at ``--instance 1``, against the same world::

   python tools/ardu_pilot.py --agent uav1 --instance 1 --location 5,0,1 ...
   sim_vehicle.py -v ArduCopter -f json:127.0.0.1 -I 1 --sysid 2 -N

Every port shifts by ten, the world is shared, and both aircraft see each other
in it -- which is the entire point, and the one thing a local simulator cannot
do.


What is still shared, and therefore still a cost
------------------------------------------------

The world renders every sensor on every tick, regardless of who subscribed.
With several pilots, **one person's cameras slow the world for everyone**, and
because the world has one clock, a slow world is slow for every vehicle in it.

Per-sensor ``Hz`` works at spawn as well as on
:class:`~biguasim.server.actions.AddSensor`, which is the main lever. Skipping
*serialisation* for topics nobody watches would help further and needs the
world to track subscriptions. Skipping the *render* is harder and unsolved.
