"""Which teleport type codes does a vehicle actually honour?

`teleport` and `set_physics_state` write the same 12-float buffer but different
type codes, and the plugin handles them separately -- so a handler written for
one is invisible to the other. This asks a vehicle for each code in turn and
reports where it ended up.

    python tools/teleport_probe.py --package Competition --world CompetionMap \
        --vehicle HolybroX500

A code that moves the actor is implemented. A code that leaves it at its spawn
point is not.

**Spawn point matters, and it is why an earlier version of this probe lied.** A
vehicle created inside geometry can fail to initialise its physics body and
then reports nothing for the rest of the run -- a HolybroX500 at (0, 0, 0) in
CompetionMap reads zeros forever, while the same vehicle at (10, 0, 3) is fine.
The teleport is still delivered and the engine still logs it; only the readback
is dead. So each case is spawned in clear air, well apart, and --spawn-z can
raise them further if a map needs it.
"""
import argparse

import numpy as np

import biguasim
from biguasim.agents import AgentDefinition
from biguasim.sensors import SensorDefinition

#: name -> what the buffer's type slot is set to, and what it means.
CODES = [
    (1, "teleport, location only"),
    (2, "teleport, rotation only"),
    (3, "teleport, location + rotation   <- the viewer's live draw"),
    (15, "set_physics_state, all four    <- soft-kill, SetPose, graveyard"),
]


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--package", default="Competition")
    parser.add_argument("--world", default="CompetionMap")
    parser.add_argument("--vehicle", default="HolybroX500")
    parser.add_argument("--against", default="DjiMatrice",
                        help="a second vehicle as a control; '' to skip")
    parser.add_argument("--target", default="7,7,5")
    parser.add_argument("--frames", type=int, default=150)
    parser.add_argument("--spawn-z", type=float, default=20.0,
                        help="height to create each actor at. Must be clear of "
                             "geometry: a vehicle spawned inside something "
                             "reports zeros forever and looks exactly like a "
                             "teleport that was ignored")
    parser.add_argument("--viewport", action="store_true")
    args = parser.parse_args()

    target = [float(v) for v in args.target.split(",")]
    kinds = [args.vehicle] + ([args.against] if args.against else [])

    env = biguasim.make(scenario_cfg={
        "package_name": args.package, "world": args.world, "main_agent": "",
        "ticks_per_sec": 100, "frames_per_sec": False,
        "octree_min": 0.1, "octree_max": 5.0, "agents": [],
    }, show_viewport=args.viewport)
    env.reset()
    env._default_state_fn = env._get_full_state

    cases = []
    for kind in kinds:
        for code, label in CODES:
            name = "t{}".format(len(cases))
            full = name + "-id0"
            # Spread out and up: two actors sharing a spawn point, or one
            # inside the ground, is the failure this probe exists to not
            # produce itself.
            env.add_agent(AgentDefinition(
                agent_name=full, agent_type=kind,
                starting_loc=(5.0 * len(cases), 0.0, args.spawn_z),
                sensors=[SensorDefinition(
                    agent_name=full, agent_type=kind,
                    sensor_name="DynamicsSensor", sensor_type="DynamicsSensor",
                    socket="IMUSocket",
                    config={"UseCOM": True, "UseRPY": False}, tick_every=1)]))
            cases.append((name, kind, code, label))

    # The buffer is written directly rather than through teleport() or
    # set_physics_state(), so each code can be exercised on its own -- those
    # two methods only ever produce 3 and 15.
    for _ in range(args.frames):
        for name, _, code, _label in cases:
            agent = env.agents.get(name + "-id0")
            if agent is None:
                continue
            np.copyto(agent._teleport_buffer[0:3], target)
            np.copyto(agent._teleport_buffer[3:6], [0.0, 0.0, 0.0])
            np.copyto(agent._teleport_buffer[6:9], [0.0, 0.0, 0.0])
            np.copyto(agent._teleport_buffer[9:12], [0.0, 0.0, 0.0])
            agent._teleport_type_buffer[0] = code
        state = env.tick()

    print("\nasked for {} every frame, {} frames\n".format(target, args.frames))
    current = None
    for name, kind, code, label in cases:
        if kind != current:
            print("  {}:".format(kind))
            current = kind
        frames = state.get(name)
        got = None
        if frames and "DynamicsSensor" in frames[0]:
            got = np.round(np.asarray(frames[0]["DynamicsSensor"],
                                      dtype=float)[6:9], 2)
        if code == 2:
            # Rotation only: the position is not supposed to change, so it
            # says nothing about whether the code is handled.
            verdict = "(position n/a)"
        elif got is None:
            verdict = "no sensor data"
        elif float(np.linalg.norm(np.asarray(got) - np.asarray(target))) < 1.0:
            verdict = "HANDLED"
        elif not np.any(got):
            # All zeros is not a refused teleport. It is an actor whose physics
            # body never came up -- usually because it was spawned inside
            # something -- and it reports nothing regardless of what is sent.
            verdict = "NO READBACK -- actor reporting zeros, raise --spawn-z"
        else:
            verdict = "ignored -- did not reach the target"
        print("    type {:<3} {:<46} -> {}  {}".format(
            code, label, got, verdict))
    print()
    env.__exit__(None, None, None)


if __name__ == "__main__":
    main()
