"""Fly one ArduPilot SITL vehicle in a world running as a service.

    python tools/ardu_pilot.py --package Competition --world CompetionMap \
        --vehicle HolybroX500 --agent uav0 --location 0,0,1

Run this **on the machine running the world**. ArduPilot's JSON backend is a
blocking lockstep handshake -- one frame in flight, and the next servo command
is computed from the state that came back -- so a round trip over a network
would cap the whole flight loop at 1/RTT. Over loopback it costs nothing, and
what crosses the network instead is MAVLink, which was built for radios.

Deliberately no ROS. This is the smallest thing that proves a served world can
fly an ArduPilot vehicle; if it does not work, there is exactly one place to
look. The ROS bridge node is the same runner with publishers attached.

It waits for SITL before spawning anything, so print the command it gives you
and start SITL in another terminal. Then point a GCS at the MAVLink port it
names.
"""
import argparse
import signal

from biguasim.ardubridge import VEHICLE_REGISTRY, RemoteArduRunner
from biguasim.client.remote import RemoteError


#: ArduPilot ships this in Tools/autotest/locations.txt as exactly the
#: latitude and longitude the bridge uses for the world origin, which is why
#: the two agree by name rather than by coincidence.
RATBEACH = (33.810313, -118.393867)


def runner_gps_origin(runner):
    """The latitude and longitude this pilot's bridge is converting around."""
    return runner._bridge._gps_origin


def _home_argument(lat, lon):
    """The sim_vehicle argument that puts SITL's home at the world origin.

    Named locations are nicer to read and to type, so one is used when it fits;
    anything else falls back to the explicit form rather than being silently
    wrong.
    """
    if (round(lat, 6), round(lon, 6)) == RATBEACH:
        return "-L RATBeach"
    return "--custom-location={},{},0,270".format(lat, lon)


def _triple(text):
    """Parse 'x,y,z' into a tuple of floats."""
    parts = [p for p in text.replace(" ", "").split(",") if p]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            "expected three comma-separated numbers, got {!r}".format(text))
    return tuple(float(p) for p in parts)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--package", default="Competition")
    parser.add_argument("--world", default="CompetionMap")
    parser.add_argument("--address", default="127.0.0.1",
                        help="where the world is. Loopback unless you have a "
                             "very good reason -- see the module docstring")
    parser.add_argument("--port", type=int, default=8770,
                        help="the world's request port; state is on port+1")
    parser.add_argument("--vehicle", default="HolybroX500",
                        choices=sorted(VEHICLE_REGISTRY),
                        help="which ArduPilot vehicle profile to fly")
    parser.add_argument("--agent", default=None,
                        help="agent name in the world; defaults from --vehicle")
    parser.add_argument("--instance", type=int, default=0,
                        help="SITL instance number. Shifts every port by 10x "
                             "this, which is how a second vehicle avoids the first")
    parser.add_argument("--location", type=_triple, default=(0.0, 0.0, 1.0),
                        metavar="X,Y,Z")
    parser.add_argument("--rotation", type=_triple, default=(0.0, 0.0, 0.0),
                        metavar="R,P,Y")
    parser.add_argument("--wait", type=float, default=120.0,
                        help="seconds to wait for SITL before giving up")
    parser.add_argument("--report", type=int, default=0,
                        metavar="N", help="print a frame/skip count every N ticks")
    parser.add_argument("--params", default=None, metavar="FILE",
                        help="a SITL parameter file, quoted back in the printed "
                             "sim_vehicle command. Use the GPS-navigating one "
                             "for a plain flight: the vision-navigating one "
                             "needs an autonomy stack to supply odometry, and "
                             "without it the vehicle will not arm")
    args = parser.parse_args()

    profile = VEHICLE_REGISTRY[args.vehicle]
    runner = RemoteArduRunner(
        profile,
        package_name=args.package, world=args.world,
        agent=args.agent, address=args.address, port=args.port,
        instance=args.instance, location=args.location, rotation=args.rotation,
    )

    print(__doc__.strip().splitlines()[0])
    print()
    print("  vehicle   {} ({}, {} motors)".format(
        args.vehicle, profile.control_abstraction, profile.num_motors))
    print("  agent     {}".format(runner.agent))
    print("  world     {}/{} at {}:{}".format(
        args.package, args.world, args.address, args.port))
    print("  fdm       udp/{}   <- start SITL pointing here".format(runner.fdm_port))
    print("  mavlink   tcp/{}   <- point a GCS here once it is up".format(
        runner.mavlink_port))
    print()
    print("The world must tick fast enough to fly. serve_world.py defaults to")
    print("--rate 20, which is fine for a viewer and hopeless for an attitude")
    print("loop -- start it with --rate 200.")
    print()
    # Derived, not written down, so the home location cannot drift away from
    # the origin the bridge converts positions around. If those two disagree
    # the EKF believes the vehicle is thousands of kilometres from home.
    lat, lon = runner_gps_origin(runner)
    home = _home_argument(lat, lon)
    print("Start SITL with:")
    print("  sim_vehicle.py -v ArduCopter -f JSON:127.0.0.1 \\")
    print("      {} --console --map \\".format(home))
    if args.instance:
        print("      -I {} --sysid {} \\".format(args.instance, args.instance + 1))
    print("      --add-param-file={}".format(
        args.params or "<your>/holybro_sitl_gps.parm"))
    print()
    print("  {} is where the bridge puts the world origin. SITL's home has to".format(
        home.split("=")[-1] if "=" in home else home.split()[-1]))
    print("  match it or the EKF places the vehicle a long way from itself.")
    print()
    print("Forward MAVLink to another machine with:")
    print("  mavproxy.py --master tcp:127.0.0.1:{} --out tcpin:0.0.0.0:{}".format(
        runner.mavlink_port, 14550 + args.instance))
    print()

    stopping = []
    signal.signal(signal.SIGINT, lambda *_: stopping.append(True))
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(True))

    try:
        runner.connect()
        print("  world ticks at {} Hz".format(runner.ticks_per_sec))
        print()
        runner.start(timeout=args.wait)
        print("flying -- ctrl-c to land the pilot (the vehicle stays, on zero throttle)")
        runner.run(should_stop=lambda: bool(stopping),
                   report_every=args.report)
    except (RuntimeError, RemoteError) as exc:
        # RemoteError is what a world that is not there looks like, and it is
        # the single most likely way to arrive here. A traceback for that would
        # bury the one line worth reading.
        raise SystemExit("pilot failed: {}".format(exc))
    finally:
        for failure in runner.failures():
            print("world rejected: {}".format(failure.get("error")))
        if runner.frames or runner.skipped:
            print("serviced {} ticks, skipped {}".format(
                runner.frames, runner.skipped))
        runner.close()


if __name__ == "__main__":
    main()
