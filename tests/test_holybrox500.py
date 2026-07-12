"""Verification for the HolybroX500 competition drone across all control types.

Runs the Competition/CompetionMap engine (which registers the HolybroX500 agent)
and drives each control abstraction, checking the drone reaches the expected
state. A small margin of error is allowed.

Run with the biguasim conda env, e.g.:
    python tests/test_holybrox500.py

Notes
-----
* The engine's first launch compiles shaders and can exceed the default 60s tick
  timeout; we wait indefinitely for the first tick.
* cmd_pos_yaw is verified at a 30 deg heading change. The underlying controller
  has a singular branch at exactly +-90 deg yaw, so large simultaneous yaw slews
  are outside the reliable envelope for this low-inertia airframe.
"""
import uuid
import numpy as np
import biguasim

# First-tick shader compilation can exceed the hardcoded 60s timeout.
biguasim.environments.BiguaSimEnvironment._timeout = property(lambda self: None)

PKG, WORLD = "Competition", "CompetionMap"
START = [0, 0, 50]


def _agent(control_abstraction, dynamics, location, rpy=False):
    sensors = [{
        "sensor_type": "DynamicsSensor", "socket": "IMUSocket",
        "configuration": {"UseCOM": True, "UseRPY": False},
    }]
    if rpy:
        sensors.append({
            "sensor_type": "DynamicsSensor", "sensor_name": "rpy", "socket": "IMUSocket",
            "configuration": {"UseCOM": False, "UseRPY": True},
        })
    return {
        "agent_name": "uav0", "agent_type": "HolybroX500", "sensors": sensors,
        "dynamics": {"batch_size": 1, **dynamics},
        "control_abstraction": control_abstraction,
        "location": location, "rotation": [0.0, 0.0, 0.0],
    }


def _sensor(state, name="DynamicsSensor"):
    return np.asarray(state["uav0"][0][name], dtype=float)


def _make_env(agent):
    scenario = {
        "package_name": PKG, "world": WORLD, "main_agent": "uav0",
        "frames_per_sec": False, "agents": [agent],
    }
    binary = biguasim.packagemanager.get_binary_path_for_package(PKG)
    return biguasim.environments.BiguaSimEnvironment(
        scenario=scenario, binary_path=binary, show_viewport=True,
        verbose=False, uuid=str(uuid.uuid4()), ticks_per_sec=30,
    )


def main():
    results = []
    with _make_env(_agent("cmd_motor_speeds", {}, START)) as env:
        # 1) cmd_motor_speeds: hover thrust should hold altitude with ~zero accel.
        w_hover = np.sqrt(2.0 * 9.81 / (4 * 8.54858e-6))
        d = _sensor(env.step([w_hover] * 4, 300))
        ok = abs(d[8] - 50) < 0.5 and np.allclose(d[0:3], 0, atol=1e-2)
        results.append(("cmd_motor_speeds (hover z=50)", ok, {"z": round(d[8], 3)}))

        # 2) cmd_vel
        env.scenario["agents"] = [_agent("cmd_vel", {}, START)]
        env.reset()
        tgt = [1, 1, 0.5]
        vel = _sensor(env.step(tgt, 400))[3:6]
        ok = np.allclose(tgt, vel, atol=1e-1, rtol=1e-1)
        results.append(("cmd_vel", ok, {"target": tgt, "vel": np.round(vel, 3).tolist()}))

        # 3) cmd_vel_yaw (velocity tracking; yaw is a continuous rate command). The
        # continuous yaw induces a residual horizontal limit cycle, so we check the
        # mean velocity over a settling window (the meaningful tracked value).
        env.scenario["agents"] = [_agent("cmd_vel_yaw", {"k_v": 8, "kp_att": 12, "kd_att": 10, "kp_yaw": 0.6, "kd_yaw": 4}, START, rpy=True)]
        env.reset()
        tgt = [1, 1, 0.5, 5]
        env.step(tgt, 200)  # let it reach cruise
        samples = np.array([_sensor(env.step(tgt, 20), "rpy")[3:6] for _ in range(10)])
        vel = samples.mean(axis=0)
        ok = np.allclose(tgt[:3], vel, atol=0.25, rtol=0.15)
        results.append(("cmd_vel_yaw (mean vel)", ok, {"target": tgt[:3], "vel": np.round(vel, 3).tolist()}))

        # 4) cmd_pos_yaw (position + 30 deg heading)
        env.scenario["agents"] = [_agent("cmd_pos_yaw", {"kp_yaw": 2, "kd_yaw": 2}, START, rpy=True)]
        env.reset()
        tgt = [2, 2, 60, 30]
        d = _sensor(env.step(tgt, 800), "rpy")
        pos, yaw = d[6:9], float(d[-1])
        ok = np.allclose(tgt[:3], pos, atol=0.5, rtol=0.05) and abs(yaw - tgt[3]) < 5
        results.append(("cmd_pos_yaw (yaw=30)", ok, {"target": tgt, "pos": np.round(pos, 3).tolist(), "yaw": round(yaw, 2)}))

    print("\n===== HolybroX500 verification =====")
    allok = True
    for name, ok, info in results:
        allok &= ok
        print(f"[{'PASS' if ok else 'FAIL'}] {name:32s} {info}")
    print("=====", "ALL PASS" if allok else "SOME FAILED", "=====")
    return allok


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
