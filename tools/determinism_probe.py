"""Determinism probe.

Records one trial of a fixed scenario and command sequence, so two trials run as
separate processes can be diffed. That is exactly what replaying an action log
later does, which is why the trials must not share an interpreter.

    python tools/determinism_probe.py --scenario contact --out a.npz
    python tools/determinism_probe.py --scenario contact --out b.npz
    python tools/determinism_probe.py --compare a.npz b.npz

Scenarios, in increasing order of how likely they are to break:

    open      one agent, open air, no contacts
    contact   one agent flown into the terrain, plus raycast sensors
    multi     three agents ticking together (engine actor ordering)
    world     a server.World driven by a scripted action stream, including a
              mid-run spawn and a mid-run sensor -- checks that routing actions
              through the queue keeps the reproducibility the others showed
"""
import argparse
import sys

import numpy as np


def _uav(name, x, sensors):
    return {
        "agent_name": name,
        "agent_type": "DjiMatrice",
        "sensors": sensors,
        "dynamics": {"batch_size": 1},
        "control_abstraction": "cmd_motor_speeds",
        "location": [x, 0, 25],
        "rotation": [0, 0, -90],
    }


DYNAMICS = {
    "sensor_type": "DynamicsSensor",
    "socket": "IMUSocket",
    "configuration": {"UseCOM": True, "UseRPY": False},
}
RANGEFINDER = {
    "sensor_type": "RangeFinderSensor",
    "socket": "IMUSocket",
    "configuration": {"LaserCount": 8, "LaserMaxDistance": 30},
}
COLLISION = {"sensor_type": "CollisionSensor", "socket": "IMUSocket"}


def build(scenario):
    """Returns (config, agent names, ticks, command function)."""
    base = {
        "package_name": "SkyDive",
        "world": "Pier-Harbor",
        "main_agent": "uav0",
        "ticks_per_sec": 20,
        "frames_per_sec": False,
        "octree_min": 0.02,
        "octree_max": 5.0,
    }

    if scenario == "open":
        def command(t, _k=0):
            b = 320.0
            return [b + 12 * np.sin(t / 9.0), b + 12 * np.cos(t / 11.0),
                    b - 9 * np.sin(t / 13.0), b - 9 * np.cos(t / 7.0)]

        return dict(base, agents=[_uav("uav0", 0, [DYNAMICS])]), ["uav0"], 200, command

    if scenario == "contact":
        def command(t, _k=0):
            # Hover, cut power into the terrain, then scrabble against it.
            b = 340.0 if t < 40 else (120.0 if t < 90 else 300.0 + 40 * np.sin(t / 5.0))
            return [b + 8 * np.sin(t / 9.0), b + 8 * np.cos(t / 11.0),
                    b - 6 * np.sin(t / 13.0), b - 6 * np.cos(t / 7.0)]

        agent = _uav("uav0", 0, [DYNAMICS, RANGEFINDER, COLLISION])
        agent["location"] = [0, 0, 12]
        return dict(base, agents=[agent]), ["uav0"], 300, command

    if scenario == "multi":
        names = ["uav0", "uav1", "uav2"]

        def command(t, k=0):
            b = 320.0 + 5 * k
            return [b + 12 * np.sin((t + k) / 9.0), b + 12 * np.cos((t + 3 * k) / 11.0),
                    b - 9 * np.sin((t + 2 * k) / 13.0), b - 9 * np.cos((t + k) / 7.0)]

        agents = [_uav(n, i * 6, [DYNAMICS]) for i, n in enumerate(names)]
        return dict(base, agents=agents), names, 200, command

    raise SystemExit("unknown scenario: " + scenario)


def script(world_module):
    """A fixed action stream for the `world` scenario.

    Deliberately submitted out of order, and from two clients, so the world's
    `(target_tick, client_id, seq)` sort is doing real work rather than
    receiving an already-sorted stream.
    """
    a = world_module
    plan = []
    for t in range(20, 180, 10):
        # Two clients, submitted in the wrong order on purpose.
        plan.append(a.SetControl(client_id="bob", seq=t, target_tick=t,
                                 agent="uav0", command=[330.0 + t % 7] * 4))
        plan.append(a.SetControl(client_id="alice", seq=t, target_tick=t,
                                 agent="uav0", command=[325.0 + t % 5] * 4))
    plan.append(a.SpawnAgent(
        client_id="alice", seq=1, target_tick=40, agent="uav9",
        agent_type="DjiMatrice", location=(10.0, 0.0, 25.0),
        sensors=[{"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
                  "configuration": {"UseCOM": True, "UseRPY": False}}]))
    for t in range(50, 180, 10):
        plan.append(a.SetControl(client_id="alice", seq=1000 + t, target_tick=t,
                                 agent="uav9", command=[318.0 + t % 9] * 4))
    plan.append(a.AddSensor(client_id="bob", seq=2, target_tick=60,
                            agent="uav0", sensor_type="RangeFinderSensor",
                            socket="IMUSocket",
                            config={"LaserCount": 4, "LaserMaxDistance": 25}))
    plan.append(a.KillAgent(client_id="alice", seq=3, target_tick=150, agent="uav9"))
    return plan


def run_world(out):
    """Drive a server.World rather than the environment directly."""
    from biguasim.server import World
    from biguasim.server import actions as world_actions

    cfg, _names, _ticks, _command = build("open")
    rows, failures = {}, []
    with World(cfg, input_delay=3) as world:
        for action in script(world_actions):
            world.submit(action)
        for _ in range(200):
            state = world.step()
            failures.extend(world.drain_errors())
            for agent in world.agents:
                for sensor, value in state.get(agent, [{}])[0].items():
                    if value is None:
                        continue
                    rows.setdefault("{}.{}".format(agent, sensor), []).append(
                        np.asarray(value, dtype=np.float64).ravel())

    for tick, action, message in failures:
        print("  tick {:3d} {:<18} {}".format(tick, action.kind, message))

    # Channels start and stop as agents and sensors come and go, so they are
    # stored per channel rather than as one rectangular block.
    np.savez(out, **{k: np.stack(v) for k, v in rows.items()})
    print("wrote {}: {} channels".format(out, len(rows)))


def run(scenario, out):
    import biguasim

    cfg, names, ticks, command = build(scenario)
    rows = {}
    with biguasim.make(scenario_cfg=cfg, show_viewport=False) as env:
        env.reset()
        for t in range(ticks):
            if len(names) == 1:
                state = env.step(command(t))
            else:
                state = env.step({n: command(t, k) for k, n in enumerate(names)})
            for n in names:
                for sensor, value in state[n][0].items():
                    if value is None:
                        continue
                    rows.setdefault(
                        "{}.{}".format(n, sensor), []
                    ).append(np.asarray(value, dtype=np.float64).ravel())

    np.savez(out, **{k: np.stack(v) for k, v in rows.items()})
    print("wrote {}: {} ticks, {} channels".format(out, ticks, len(rows)))


def compare(path_a, path_b):
    a, b = np.load(path_a), np.load(path_b)
    identical = True
    for key in sorted(a.files):
        x, y = a[key], b[key]
        if np.array_equal(x, y):
            print("  [BIT-IDENTICAL] {}  {}".format(key, x.shape))
            continue
        identical = False
        delta = np.abs(x - y)
        first = np.argwhere(delta.any(axis=1)).ravel()[0]
        print("  [DIVERGES]      {} first tick {}, max |delta| {:.3e}".format(
            key, first, delta.max()))

    print("\n=> " + ("deterministic: re-execution replay is valid"
                     if identical else
                     "NOT deterministic: replay needs recorded state, not just actions"))
    return 0 if identical else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="contact",
                        choices=["open", "contact", "multi", "world"])
    parser.add_argument("--out")
    parser.add_argument("--compare", nargs=2, metavar=("A", "B"))
    args = parser.parse_args()

    if args.compare:
        sys.exit(compare(*args.compare))
    if not args.out:
        parser.error("--out is required unless --compare is given")
    if args.scenario == "world":
        run_world(args.out)
    else:
        run(args.scenario, args.out)
