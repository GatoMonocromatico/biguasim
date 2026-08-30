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


Rates are asked for, not configured
-----------------------------------

The pilot takes the world's tick rate from the greeting it gets on connect, and
sizes the IMU to match -- one sample per tick, which is what ArduPilot's EKF
expects. Nothing about that rate is a flag or a YAML entry on the pilot's side,
because a number the caller supplies is a number the caller can get wrong, and
the symptom appears much later: a refused spawn quoting a rate nobody typed.

An extra sensor asking for a rate the world cannot produce is rejected at
connect rather than at spawn, with both numbers named. Sensor rates are tick
dividers, so a rate must divide the tick rate exactly and cannot exceed it.


Flying one, with no ROS involved
--------------------------------

Three terminals on the machine running the world. First the world -- and note
the rate, which is the whole reason this section names it::

   python tools/serve_world.py --package Competition --world CompetionMap \
       --port 8770 --rate 200

.. important::

   ``serve_world.py`` defaults to ``--rate 20``. That is a sensible default for
   a world people are watching and hopeless for one being flown: the tick rate
   *is* the rate ArduPilot's attitude loop runs at, and a multirotor at 20 Hz
   will not arm, let alone hold attitude.

   The pilot asks the world for its rate on connect and warns when it is this
   low, rather than letting it look like a flight-tuning problem later.

Then the pilot, which prints the ports it wants and waits::

   python tools/ardu_pilot.py --package Competition --world CompetionMap \
       --vehicle HolybroX500 --agent uav0 --location 0,0,1

Then SITL, pointed at the port the pilot named. The pilot prints this command
for you, filled in, so it cannot drift out of step with the bridge::

   sim_vehicle.py -v ArduCopter -f JSON:127.0.0.1 \
       -L RATBeach --console --map \
       --add-param-file=/path/to/holybro_sitl_gps.parm

.. important::

   **``-L RATBeach`` is not optional.** ArduPilot ships that location as
   ``33.810313,-118.393867``, which is exactly the origin the bridge converts
   world positions around -- the two agree by construction, not by luck. Leave
   it out and SITL's home is somewhere else entirely, so the EKF places the
   vehicle thousands of kilometres from where the world says it is.

   If a pilot is given a different ``gps_origin``, it prints the equivalent
   ``--custom-location`` instead. Take the command from the pilot rather than
   from here.

.. important::

   **Which parameter file, and why it decides whether it arms.**

   A tuned frame usually comes with two, and they are not interchangeable:

   * one where the EKF navigates on **visual odometry** -- ``GPS1_TYPE 0``,
     ``VISO_TYPE 1``, ``EK3_SRC1_POSXY 6``. That is the competition
     configuration, and it needs the autonomy stack running to supply the
     odometry. With no ROS anywhere, the EKF has no horizontal position source
     and the vehicle will not arm in any position-holding mode.
   * one where it navigates on **GPS** -- ``GPS1_TYPE 1``, ``VISO_TYPE 0``,
     ``EK3_SRC1_POSXY 3``. SITL synthesises the GPS from the latitude and
     longitude the bridge sends it, so nothing else is required.

   **Use the GPS one for a plain SITL flight.** The vision configuration is
   for later, once the autonomy stack is in the picture.

   Note also ``SCHED_LOOP_RATE``: arming requires the gyro rate -- which here
   *is* the world's tick rate -- to be at least ``1.8 x SCHED_LOOP_RATE``. At
   the usual 100 that means the world must tick at 180 or more, which is the
   other reason ``--rate 200`` above is not arbitrary.

The pilot spawns ``uav0`` the moment SITL says hello.

Watch it from anywhere with ``tools/watch_world.py``, which needs nothing from
the pilot: the vehicle is in the world like any other.

.. warning::

   **Give a HolybroX500 room when it is created.** That airframe rests at
   0.216 m and fails below it: created any lower it sits inside the ground,
   its physics body never initialises, and its sensors report zeros for the
   rest of the run. Teleports still arrive and the engine still logs them --
   the actor is simply never usable, which looks like a great many other
   things before it looks like a spawn point.

   ``--location 0,0,1`` is clear. ``--location 0,0,0`` is not.

   The viewer creates its puppets lifted clear for the same reason, so this
   only applies to where you ask the *pilot* to spawn.


Reaching SITL from a GCS on another machine
-------------------------------------------

``sim_vehicle.py`` **already starts MAVProxy**, connected to SITL with
``--master tcp:127.0.0.1:5760``. That matters twice over:

* SITL's MAVLink TCP port accepts one client, and MAVProxy is holding it. A
  second ``mavproxy.py --master tcp:127.0.0.1:5760`` will not get in.
* MAVProxy's only default output is ``--out 127.0.0.1:14550`` -- **localhost**.
  Which is exactly why QGroundControl on another machine sees nothing at all.

So add an output to the MAVProxy that is already running, on the
``sim_vehicle.py`` command line. Either direction works; they differ only in
who has to know whose address.

**The server listens, and QGC dials in.** Nothing needs to know where you are,
so it survives your address changing::

   sim_vehicle.py ... --out=tcpin:0.0.0.0:14551

Then in QGroundControl: *Application Settings, Comm Links, Add*, type **TCP**,
host the server's tailnet name, port ``14551``. Connect.

**Or the server pushes, and QGC listens.** QGC auto-connects to UDP on 14550,
so there is nothing to configure at all -- but the server has to be told your
address::

   sim_vehicle.py ... --out=udp:<your-tailscale-ip>:14550

Get that address with ``tailscale ip -4`` on the machine running QGC.

.. note::

   ``0.0.0.0`` binds every interface. On a host reachable only through a
   tailnet that is fine; on one with a public address, bind the tailnet address
   instead -- ``--out=tcpin:100.x.y.z:14551`` -- since nothing in MAVLink
   authenticates anybody.

Both forms are additive: MAVProxy keeps its console, its map and its localhost
output, and simply sends to one more place.


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
