"""Inspect or re-run a recorded world.

    python tools/replay.py run.bslog                 # what is in it
    python tools/replay.py run.bslog --verify        # re-run and compare

Verifying re-executes the recording and checks it against the keyframes stored
alongside. Because the simulator is reproducible, a clean verify means the
recording is a complete account of the run -- and that editing an action and
re-running it would tell you what would have happened instead.
"""
import argparse
import sys

from biguasim.server.recording import Recording, current_device, replay


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path")
    parser.add_argument("--verify", action="store_true",
                        help="re-run it and compare against the keyframes")
    parser.add_argument("--ticks", type=int, help="stop after this many")
    parser.add_argument("--any-device", action="store_true",
                        help="replay a recording made on another device anyway")
    args = parser.parse_args()

    recording = Recording(args.path)
    header = recording.header
    print("{}\n  build      {}\n  device     {}\n  scenario   {} / {}".format(
        args.path, header["build"], recording.device,
        recording.scenario.get("package_name"), recording.scenario.get("world")))
    print("  actions    {}\n  keyframes  {}\n  last tick  {}".format(
        len(recording.actions), len(recording.keyframes), recording.last_tick))

    kinds = {}
    for _tick, action, _error in recording.actions:
        kinds[action.kind] = kinds.get(action.kind, 0) + 1
    if kinds:
        print("  by kind    " + ", ".join(
            "{} x{}".format(k, n) for k, n in sorted(kinds.items())))

    failed = [(t, a, e) for t, a, e in recording.actions if e]
    if failed:
        print("  failed     {} (these should fail again on replay)".format(len(failed)))

    warning = recording.check_device(strict=False)
    if warning:
        print("\n  WARNING: {}".format(warning))

    if not args.verify:
        return 0

    print("\nreplaying on {}...".format(current_device()))
    result = replay(recording, ticks=args.ticks,
                    strict_device=not args.any_device)
    print("  compared {} keyframes, matched {}".format(
        result["compared"], result["matched"]))
    for tick, agent, delta in result["divergences"][:10]:
        print("    tick {:5d}  {:<12} max |delta| {:.3e}".format(tick, agent, delta))

    if result["divergences"]:
        print("\nDIVERGED -- this recording does not reproduce its own run")
        return 1
    print("\nreproduced exactly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
