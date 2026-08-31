.. _ardupilot-pilot:

===================================
Flying ArduPilot in a Served World
===================================

A world can hold vehicles it integrates itself, driven by
:meth:`~biguasim.client.remote.RemoteWorld.set_control`. This page covers the
other kind: a vehicle flown by a real ArduPilot, with its EKF, its flight modes
and a ground station you can point at it.

The piece that makes it work is a **pilot**: one process per vehicle, holding a
SITL, an FDM bridge and a connection to the world. It is
:class:`~biguasim.ardubridge.RemoteArduRunner`. The world can start one for you,
or you can start one yourself.


Where SITL Has To Run
=====================

**On the machine running the world.** This is a constraint, not a preference.

ArduPilot's JSON backend is a lockstep protocol. ``JSON::update()`` is
``output_servos()`` followed by a blocking ``recv_fdm()`` -- one frame in flight
-- and the servos for frame N+1 are computed from the state that came back for
frame N. That is a control dependency, so nothing about it can be pipelined.
Put a network in the middle and the whole flight loop is capped at ``1/RTT``:
at 45 ms that is 22 Hz, and a multirotor does not fly at 22 Hz.

Over loopback the same loop runs at about 0.05 ms. What crosses the network
instead is MAVLink, which was designed for telemetry radios.

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

The general rule: anything with a serial round-trip dependency stays beside the
world; anything that streams, or was built for a lossy link, can cross the
network.


Letting the World Start It
==========================

The world already knows which world, which vehicle and where, so it can compose
the rest. Start it with the feature enabled::

   python tools/serve_world.py --package Competition --world CompetionMap \
       --port 8770 --rate 200 --allow-sitl \
       --sim-vehicle ~/ardupilot/Tools/autotest/sim_vehicle.py \
       --sitl-params-dir ~/params \
       --sitl-log-dir ~/sitl-logs

Then, from any machine:

.. code-block:: python

   from biguasim.client import RemoteWorld

   scenario = {"package_name": "Competition", "world": "CompetionMap"}

   with RemoteWorld(address="fakenatty", port=8770, scenario_cfg=scenario) as world:
       info = world.spawn_ardupilot_agent(
           "uav0", "HolybroX500",
           location=(0.0, 0.0, 1.0),
           ardupilot={"params": "holybro_sitl_gps.parm", "console": True})

       print(info["ports"]["gcs"])      # point QGroundControl at this

The world starts SITL and a pilot beside itself and replies with the ports. The
agent appears a moment later, created by that pilot once its flight controller
is up -- not by the request itself. See `The Vehicle And Its Pilot`_.

.. note::

   ``--sim-vehicle`` is usually required. ``sim_vehicle.py`` is rarely on
   ``PATH``; without the flag the world reports that it could not start the
   vehicle.

.. warning::

   ``--allow-sitl`` is off by default and should stay off on anything exposed.
   It runs processes on behalf of whoever can reach the port, and that port is
   not authenticated.

   Parameter files resolve inside ``--sitl-params-dir`` and nowhere else. The
   path is resolved before it is checked, so neither ``..`` nor a symlink
   escapes it. Without the flag, parameter files are refused entirely.


What The World Fills In
-----------------------

These are derived and cannot be overridden. Passing one is refused rather than
merged, because two homes or two instances -- silently -- produce a vehicle that
looks broken for reasons unrelated to the flag.

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Flag
     - Source
   * - ``-v``
     - The vehicle's registry entry. A BlueROV2 is ArduSub, a HolybroX500 is
       ArduCopter.
   * - ``-f JSON:``
     - The bridge the world is about to start.
   * - ``-L``
     - The bridge's GPS origin. See `Home And The GPS Origin`_.
   * - ``-I``
     - The lowest free instance, which is the whole of port allocation.
   * - ``--out``
     - A listening MAVLink endpoint, so a ground station elsewhere can connect
       without the world knowing its address.

Everything else is yours. A key this code does not recognise becomes a flag::

   ardupilot={"console": True, "map": True, "speedup": 2, "wipe_eeprom": True}
   # --console --map --speedup 2 --wipe-eeprom

``True`` is a bare flag, ``False`` and ``None`` are omitted, a list repeats the
flag, and underscores become dashes. The command is built as an argument list
and run without a shell, so a value containing shell metacharacters is an
argument and nothing more.

Killing the agent stops its SITL and its pilot, and so does shutting the world
down. The whole process group is signalled, because ``sim_vehicle.py`` is a
launcher and what needs to stop is everything it started.


Starting It By Hand
===================

Useful when tuning ``sim_vehicle.py`` options, or on a world without
``--allow-sitl``. Three terminals on the machine running the world.

First the world::

   python tools/serve_world.py --package Competition --world CompetionMap \
       --port 8770 --rate 200

Then the pilot, which prints the ports it wants and waits::

   python tools/ardu_pilot.py --package Competition --world CompetionMap \
       --vehicle HolybroX500 --agent uav0 --location 0,0,1

Then SITL. The pilot prints this command filled in, which is the copy to use --
it is derived from the bridge and cannot drift out of step with it::

   sim_vehicle.py -v ArduCopter -f JSON:127.0.0.1 \
       -L RATBeach --console --map \
       --add-param-file=/path/to/holybro_sitl_gps.parm

The pilot spawns ``uav0`` as soon as SITL connects.

.. note::

   ``tools/ardu_pilot.py`` is a wrapper around
   :mod:`biguasim.ardubridge.pilot_cli`. The world runs the same module when it
   starts a pilot itself, so both paths share one implementation.


Home And The GPS Origin
-----------------------

``-L RATBeach`` is not optional. ArduPilot ships that location as
``33.810313,-118.393867``, which is exactly the origin the bridge converts world
positions around; the two agree by construction. Omit it and SITL's home is
somewhere else, so the EKF places the vehicle thousands of kilometres from where
the world says it is.

A pilot given a different ``gps_origin`` prints the equivalent
``--custom-location`` instead.


Parameter Files
---------------

A tuned frame usually has two, and they are not interchangeable:

- **Visual odometry** -- ``GPS1_TYPE 0``, ``VISO_TYPE 1``,
  ``EK3_SRC1_POSXY 6``. The competition configuration. It needs the autonomy
  stack running to supply odometry; without it the EKF has no horizontal
  position source and the vehicle will not arm in any position-holding mode.
- **GPS** -- ``GPS1_TYPE 1``, ``VISO_TYPE 0``, ``EK3_SRC1_POSXY 3``. SITL
  synthesises the GPS from the latitude and longitude the bridge sends it, so
  nothing else is required.

Use the GPS one for a plain SITL flight.


The Vehicle And Its Pilot
=========================

:meth:`~biguasim.ardubridge.RemoteArduRunner.start` binds the FDM socket, waits
for SITL's opening servo packet, and only then spawns the agent.

That ordering matters because of a difference between the two bridges. The local
bridge cannot simulate without ArduPilot at all -- ``env.step()`` is only called
when a servo frame arrives. A served world free-runs on its own loop, so an
agent spawned before its flight controller is up is integrated with whatever
:class:`~biguasim.server.actions.SetControl` last said. For a quadrotor that
means falling while SITL boots.

ArduPilot spends the spawn blocked in ``recv_fdm``, which costs nothing:
``time_now_us`` only advances on a received frame.

:meth:`~biguasim.ardubridge.RemoteArduRunner.close` is the symmetric half and
retires the agent. For the case where it never runs -- a pilot that crashes --
:meth:`start` also registers zero-throttle defaults, so the vehicle outlives its
pilot on a safe standing order rather than on whatever throttle it last had. See
:ref:`world-running`.

.. warning::

   An agent name is never reusable. ``kill_agent`` is a soft kill: the world
   stops simulating the agent and drops it from the roster, but nothing in the
   UE5 plugin destroys an actor, so the engine keeps the name. A restarted pilot
   needs a new name. Until a ``DespawnAgent`` command exists in the plugin this
   cannot be fixed from Python, and the runner settles for explaining the
   engine's otherwise-baffling ``Duplicate agent name``.


Spawn Height
------------

Give the vehicle room. An actor created inside geometry never initialises its
physics body, and then reports zeros for the rest of the run while teleports
keep arriving and keep being logged -- so it looks like a great many things
before it looks like a spawn point.

A HolybroX500 rests at 0.216 m and fails below it. ``location=(0, 0, 1)`` is
clear; ``location=(0, 0, 0)`` is not.


Ports
=====

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
   * - GCS endpoint
     - 14551
     - ``14551 + 10N``
     - MAVProxy, listening
   * - World requests
     - 8770
     - 8770
     - the world -- shared
   * - World state
     - 8771
     - 8771
     - the world -- shared


Connecting A Ground Station
===========================

``sim_vehicle.py`` starts MAVProxy itself, connected to SITL with
``--master tcp:127.0.0.1:5760``. Two consequences:

- SITL's MAVLink TCP port accepts one client, and MAVProxy is holding it. A
  second ``mavproxy.py --master tcp:127.0.0.1:5760`` will not get in.
- MAVProxy's only default output is ``--out 127.0.0.1:14550``, which is
  localhost. This is why QGroundControl on another machine sees nothing.

When the world starts SITL it adds a listening endpoint for you, reported as
``info["ports"]["gcs"]``. In QGroundControl: *Application Settings*, *Comm
Links*, *Add*, type **TCP**, host the server, that port.

Starting SITL by hand, add the same output::

   sim_vehicle.py ... --out=tcpin:0.0.0.0:14551

``tcpin`` makes MAVProxy listen rather than dial, so the server never needs to
know where you are and the link survives your address changing. The alternative
is ``--out=udp:<your-address>:14550``, which QGroundControl auto-connects to
with no configuration but requires the server to know your address.

.. note::

   ``0.0.0.0`` binds every interface. On a host reachable only through a tailnet
   that is fine; on one with a public address, bind the tailnet address instead,
   since nothing in MAVLink authenticates anybody.


Tick Rate
=========

The pilot takes the world's tick rate from the greeting it gets on connect and
sizes the IMU to match, one sample per tick. Nothing about that rate is a flag
on the pilot's side: a number the caller supplies is a number the caller can get
wrong, and the symptom appears much later as a refused spawn quoting a rate
nobody typed.

An extra sensor asking for a rate the world cannot produce is rejected at
connect, naming both numbers. Sensor rates are tick dividers, so a rate must
divide the tick rate exactly and cannot exceed it.

.. warning::

   ``serve_world.py`` defaults to ``--rate 20``. That suits a world people are
   watching but not one being flown: the tick rate *is* the rate ArduPilot's
   attitude loop runs at, and arming requires the gyro rate to be at least
   ``1.8 x SCHED_LOOP_RATE``. At the usual 100 that means 180 or more.

   Use ``--rate 200``. The pilot warns when the rate is lower.


Several Vehicles
================

A second pilot at instance 1, against the same world:

.. code-block:: python

   world.spawn_ardupilot_agent("uav1", "HolybroX500", location=(5.0, 0.0, 1.0),
                               ardupilot={"params": "holybro_sitl_gps.parm"})

or by hand::

   python tools/ardu_pilot.py --agent uav1 --instance 1 --location 5,0,1 ...
   sim_vehicle.py -v ArduCopter -f JSON:127.0.0.1 -I 1 --sysid 2 -N

Every port shifts by ten, the world is shared, and both aircraft see each other
in it -- which a local simulator cannot do.


Shared Costs
============

The world renders every sensor on every tick, regardless of who subscribed. With
several pilots, one person's cameras slow the world for everyone, and because
the world has one clock, a slow world is slow for every vehicle in it.

Per-sensor ``Hz`` works at spawn as well as on
:class:`~biguasim.server.actions.AddSensor`, which is the main lever. Skipping
serialisation for topics nobody watches would help further and needs the world
to track subscriptions. Skipping the render is harder and unsolved.
