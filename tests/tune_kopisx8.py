"""Gain tuning harness for the KopisX8.

Flies a step command and reports the response, so you can iterate on the
control gains instead of eyeballing the viewport.

Two plants, same controller:

  --offline (default)  Integrates in pure Python. Legitimate because the engine
                       only INTEGRATES what the client computes: KopisX8.cpp
                       applies v_dot/w_dot with bAccelChange=true, so Unreal
                       adds no dynamics of its own. Runs in milliseconds, so
                       you can sweep. No collisions, no sensor noise.

  --engine             The real Competition/CompetionMap build. Slower, and
                       needs "KopisX8" in the HolodeckGameModeBP AgentBpMap.

Examples
--------
    # baseline, all maneuvers
    python tests/tune_kopisx8.py

    # try a stiffer attitude loop
    python tests/tune_kopisx8.py --set kp_att=14 --set kd_att=9

    # sweep a grid and rank by settling time
    python tests/tune_kopisx8.py --sweep kp_att=8,10,14 --sweep kd_att=4,6,9

    # confirm the winner in the simulator
    python tests/tune_kopisx8.py --engine --set kp_att=14 --set kd_att=9
"""
import argparse
import itertools
import uuid

import numpy as np
import roma
import torch

from biguasim.dynamics.agents import ModelsFactory

AGENT = "KopisX8"
DT = 1.0 / 30.0          # matches ticks_per_sec=30 in the engine scenarios
G = 9.80665


# ---------------------------------------------------------------------------
# Maneuvers: (name, abstraction, command, seconds, readout, target)
# readout picks the tracked signal out of the state dict.
# ---------------------------------------------------------------------------
MANEUVERS = [
    ("climb 1 m/s",   "cmd_vel",     [0.0, 0.0, 1.0],        6.0, lambda s: s["v"][2],   1.0),
    ("fwd 2 m/s",     "cmd_vel",     [2.0, 0.0, 0.0],        6.0, lambda s: s["v"][0],   2.0),
    ("yaw 90 deg",    "cmd_pos_yaw", [0.0, 0.0, 50.0, 90.0], 14.0, lambda s: s["yaw"],  90.0),
    ("goto x=5",      "cmd_pos_yaw", [5.0, 0.0, 50.0, 0.0],  14.0, lambda s: s["x"][0],  5.0),
]


def _yaw_of(q):
    """Yaw in degrees from an xyzw quaternion."""
    x, y, z, w = q
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _metrics(trace, target, dt):
    """Overshoot %, settling time to +-5%, and final error."""
    trace = np.asarray(trace, dtype=float)
    span = abs(target) if abs(target) > 1e-9 else 1.0
    overshoot = (trace.max() - target) / span * 100 if target >= 0 else (target - trace.min()) / span * 100
    band = 0.05 * span
    outside = np.where(np.abs(trace - target) > band)[0]
    settle = (outside[-1] + 1) * dt if len(outside) and outside[-1] + 1 < len(trace) else float("nan")
    return {"overshoot_%": round(float(max(overshoot, 0.0)), 1),
            "settle_s": round(float(settle), 2) if settle == settle else None,
            "final_err": round(float(trace[-1] - target), 3)}


# ---------------------------------------------------------------------------
# Offline plant
# ---------------------------------------------------------------------------
def run_offline(params, abstraction, command, seconds, readout, start_z=50.0):
    model = ModelsFactory.build_model(AGENT)(
        batch_size=1, device="cpu", control_abstraction=abstraction, params=params)

    x = np.array([0.0, 0.0, start_z])
    v = np.zeros(3)
    w = np.zeros(3)                      # world frame, rad/s
    q = np.array([0.0, 0.0, 0.0, 1.0])   # xyzw

    trace, t = [], 0.0
    for _ in range(int(seconds / DT)):
        d = np.zeros(19)
        d[3:6], d[6:9], d[12:15], d[15:19] = v, x, w, q
        t += DT
        # step() returns [v_dot (world), w_dot (world)] -- the same 6 floats
        # the engine receives and integrates.
        acc = np.asarray(model.step([{"DynamicsSensor": d.tolist()}], [command], t)[0], dtype=float)

        v = v + acc[0:3] * DT
        x = x + v * DT
        w = w + acc[3:6] * DT
        dq = roma.rotvec_to_unitquat(torch.tensor(w * DT).double())
        q = roma.quat_product(dq, torch.tensor(q).double()).numpy()
        q = q / np.linalg.norm(q)

        trace.append(readout({"x": x, "v": v, "w": w, "yaw": _yaw_of(q)}))
    return trace


# ---------------------------------------------------------------------------
# Engine plant
# ---------------------------------------------------------------------------
def run_engine(overrides, abstraction, command, seconds, readout, start_z=50.0):
    import biguasim
    biguasim.environments.BiguaSimEnvironment._timeout = property(lambda self: None)

    agent = {
        "agent_name": "uav0", "agent_type": AGENT,
        # The scenario validator requires a sensor under the default name
        # DynamicsSensor (environments.py:366); "rpy" is the extra one we read.
        "sensors": [
            {"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
             "configuration": {"UseCOM": True, "UseRPY": False}},
            {"sensor_type": "DynamicsSensor", "sensor_name": "rpy",
             "socket": "IMUSocket",
             "configuration": {"UseCOM": False, "UseRPY": True}},
        ],
        "dynamics": {"batch_size": 1, **overrides},
        "control_abstraction": abstraction,
        "location": [0.0, 0.0, start_z], "rotation": [0.0, 0.0, 0.0],
    }
    scenario = {"package_name": "Competition", "world": "CompetionMap",
                "main_agent": "uav0", "frames_per_sec": False, "agents": [agent]}
    binary = biguasim.packagemanager.get_binary_path_for_package("Competition")

    trace = []
    with biguasim.environments.BiguaSimEnvironment(
            scenario=scenario, binary_path=binary, show_viewport=True,
            verbose=False, uuid=str(uuid.uuid4()), ticks_per_sec=30) as env:
        for _ in range(int(seconds / DT)):
            d = np.asarray(env.step(command, 1)["uav0"][0]["rpy"], dtype=float)
            trace.append(readout({"x": d[6:9], "v": d[3:6], "w": d[12:15], "yaw": float(d[-1])}))
    return trace


# ---------------------------------------------------------------------------
def evaluate(overrides, use_engine):
    base = ModelsFactory.build_model(AGENT)._params
    params = {**base, **overrides}
    rows = []
    for name, abstraction, command, seconds, readout, target in MANEUVERS:
        if use_engine:
            trace = run_engine(overrides, abstraction, command, seconds, readout)
        else:
            trace = run_offline(params, abstraction, command, seconds, readout)
        rows.append((name, target, _metrics(trace, target, DT)))
    return rows


def _score(rows):
    """Lower is better: settle time, plus penalties for overshoot and offset."""
    total = 0.0
    for _, target, m in rows:
        span = abs(target) if abs(target) > 1e-9 else 1.0
        total += (m["settle_s"] if m["settle_s"] is not None else 30.0)
        total += m["overshoot_%"] / 10.0
        total += abs(m["final_err"]) / span * 10.0
    return total


def _kv(pairs):
    out = {}
    for p in pairs or []:
        k, _, v = p.partition("=")
        out[k] = float(v)
    return out


def main():
    ap = argparse.ArgumentParser(description="KopisX8 gain tuning")
    ap.add_argument("--engine", action="store_true", help="run against the simulator")
    ap.add_argument("--set", action="append", metavar="GAIN=VAL", help="override one gain")
    ap.add_argument("--sweep", action="append", metavar="GAIN=A,B,C", help="grid-search one gain")
    args = ap.parse_args()

    fixed = _kv(args.set)
    grid = {}
    for s in args.sweep or []:
        k, _, vals = s.partition("=")
        grid[k] = [float(v) for v in vals.split(",")]

    base = ModelsFactory.build_model(AGENT)._params
    tunable = ("k_v", "kp_att", "kd_att", "kp_yaw", "kd_yaw", "kp_pos", "kd_pos", "yaw_slew_max")
    for k in list(fixed) + list(grid):
        if k not in base:
            raise SystemExit(f"unknown gain {k!r}; tunable: {', '.join(tunable)}")

    plant = "ENGINE" if args.engine else "OFFLINE"
    print(f"\nKopisX8 tuning [{plant}]   baseline: "
          + "  ".join(f"{k}={base[k]}" for k in tunable if k in base))

    if not grid:
        rows = evaluate(fixed, args.engine)
        print(f"\n  overrides: {fixed or '(baseline)'}")
        print(f"  {'maneuver':<14} {'target':>8} {'overshoot':>10} {'settle':>8} {'final err':>10}")
        for name, target, m in rows:
            st = "-" if m["settle_s"] is None else f"{m['settle_s']}s"
            print(f"  {name:<14} {target:>8.1f} {m['overshoot_%']:>9.1f}% {st:>8} {m['final_err']:>10.3f}")
        print(f"\n  score {_score(rows):.2f}  (lower is better)")
        return

    if args.engine:
        raise SystemExit("--sweep needs the offline plant; drop --engine")

    keys = list(grid)
    results = []
    for combo in itertools.product(*(grid[k] for k in keys)):
        ov = {**fixed, **dict(zip(keys, combo))}
        rows = evaluate(ov, False)
        results.append((_score(rows), ov, rows))
    results.sort(key=lambda r: r[0])

    print(f"\n  {len(results)} combinations, best first:\n")
    print(f"  {'score':>7}  " + "  ".join(f"{k:>10}" for k in keys)
          + "   per-maneuver settle_s")
    for score, ov, rows in results:
        settles = " ".join("-" if m["settle_s"] is None else f"{m['settle_s']:>5.2f}"
                           for _, _, m in rows)
        print(f"  {score:>7.2f}  " + "  ".join(f"{ov[k]:>10.4g}" for k in keys) + f"   {settles}")
    print(f"\n  best: " + "  ".join(f"--set {k}={results[0][1][k]:g}" for k in keys))


if __name__ == "__main__":
    main()
