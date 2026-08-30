"""Run a BiguaSim world as a service.

    python tools/serve_world.py --world Pier-Harbor --package SkyDive

Clients connect with biguasim.client.RemoteWorld. The world starts empty unless
--agent is given; agents are normally spawned by whoever wants them.

Pacing comes from --fps, and defaults to matching the tick rate. Left
free-running the engine ticks as fast as it can and every viewer falls
permanently behind, so real time is the right default for a shared world; pass
--fps 0 for batch runs that should go as fast as possible.
"""
import argparse
import signal

from biguasim.server.service import WorldService

DYNAMICS = {"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
            "configuration": {"UseCOM": True, "UseRPY": False}}


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--package", default="SkyDive")
    parser.add_argument("--world", default="Pier-Harbor")
    parser.add_argument("--port", type=int, default=8770,
                        help="requests here, published state on port+1")
    parser.add_argument("--bind", default="*")
    parser.add_argument("--rate", type=int, default=20, help="ticks per second")
    parser.add_argument("--fps", type=int, default=None,
                        help="wall-clock cap; defaults to --rate, 0 for uncapped")
    parser.add_argument("--input-delay", type=int, default=3,
                        help="ticks between a client submitting and it landing")
    parser.add_argument("--agent", action="append", default=[],
                        metavar="NAME:TYPE", help="agent to start with; repeatable")
    parser.add_argument("--admin", action="append", default=[],
                        help="client id exempt from ownership; repeatable")
    parser.add_argument("--viewport", action="store_true",
                        help="show the engine window on this machine")
    parser.add_argument("--ipv4-only", action="store_true",
                        help="refuse IPv6. On by default because ZeroMQ ignores "
                             "IPv6 unless told otherwise, and the wildcard bind "
                             "is dual-stack, so allowing it costs nothing")
    args = parser.parse_args()

    fps = args.rate if args.fps is None else args.fps
    agents = []
    for spec in args.agent:
        name, _, kind = spec.partition(":")
        agents.append({
            "agent_name": name, "agent_type": kind or "DjiMatrice",
            "sensors": [DYNAMICS], "dynamics": {"batch_size": 1},
            "control_abstraction": "cmd_motor_speeds",
            "location": [0, 0, 25], "rotation": [0, 0, 0],
        })

    scenario = {
        "package_name": args.package, "world": args.world,
        "main_agent": agents[0]["agent_name"] if agents else "",
        "ticks_per_sec": args.rate, "frames_per_sec": fps if fps else False,
        "octree_min": 0.02, "octree_max": 5.0,
        "agents": agents,
    }

    service = WorldService(scenario, port=args.port, bind=args.bind,
                           input_delay=args.input_delay,
                           admin_clients=set(args.admin),
                           ipv6=not args.ipv4_only,
                           show_viewport=args.viewport)

    signal.signal(signal.SIGINT, lambda *_: service.stop())
    signal.signal(signal.SIGTERM, lambda *_: service.stop())

    print("{}/{} serving on {}:{} (state on {}), {} Hz{}{}".format(
        args.package, args.world, args.bind, args.port, args.port + 1,
        args.rate, "" if fps else ", uncapped",
        "" if args.ipv4_only else ", IPv4 and IPv6"))
    if not args.ipv4_only:
        print("note: this protocol has no authentication -- client_id is "
              "whatever a client claims. Keep it to a trusted or overlay "
              "network rather than a public address.")
    print("agents at start: {}".format(service.world.agents or "none"))
    try:
        service.run()
    finally:
        service.close()
        print("\nstopped at tick {}".format(service.world.tick))


if __name__ == "__main__":
    main()
