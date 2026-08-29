"""Watch a running world in your own engine.

    python tools/watch_world.py --port 8770

Opens a local copy of the world and puts it in puppet mode: it simulates
nothing, and every frame it moves each vehicle to where the world says it is.

The pixels are made here, so watching costs the world only the few kilobytes a
tick it already publishes, however many people are looking. Where you point the
camera is nobody else's business and never leaves this machine. If you want
something the world has to actually compute -- a photoreal camera feed, a sonar
return -- ask for a sensor instead; that one costs the world a render, and
visibly so.
"""
import argparse

from biguasim.client import Viewer


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--package", default="SkyDive")
    parser.add_argument("--world", default="Pier-Harbor")
    parser.add_argument("--address", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--name", default="viewer", help="how to identify")
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--delay", type=float, default=1.5,
                        help="snapshot intervals to draw behind; more is "
                             "smoother over a worse connection")
    parser.add_argument("--seconds", type=float, help="stop after this long")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    scenario = {"package_name": args.package, "world": args.world,
                "ticks_per_sec": 20, "frames_per_sec": False,
                "octree_min": 0.02, "octree_max": 5.0, "agents": []}

    viewer = Viewer(scenario, address=args.address, port=args.port,
                    delay=args.delay, client_id=args.name,
                    show_viewport=not args.headless)
    try:
        info = viewer.connect()
        print("watching {}/{} at tick {}; agents: {}".format(
            args.package, args.world, info["tick"], info["agents"] or "none"))
        viewer.run(seconds=args.seconds, fps=args.fps)
    finally:
        print("saw up to tick {}, drew {}".format(viewer.tick, sorted(viewer.puppets)))
        viewer.close()


if __name__ == "__main__":
    main()
