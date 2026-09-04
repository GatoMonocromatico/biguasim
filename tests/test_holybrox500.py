"""Verification for the HolybroX500 competition drone across all control types.

Runs the Competition/CompetionMap engine (the build that registers the HolybroX500
agent) and drives each control abstraction, checking the drone reaches the expected
state. A small margin of error is allowed.

Run with the biguasim conda env:
    python tests/test_holybrox500.py

Operational note (cmd_pos_yaw)
------------------------------
Translation and large rotation are done as SEPARATE phases, each from a stabilized
state -- this is the reliable envelope for this low-inertia airframe:
  * translate to a waypoint while holding heading 0, then
  * rotate in place (position held) to the desired heading (any angle, incl. 180).
Doing a large yaw *while* translating (or translating while yawed 90 deg) couples
the loops and is not reliable.
"""
import uuid
import numpy as np
import biguasim

# First-tick shader compilation can exceed the hardcoded 60s timeout.
biguasim.environments.BiguaSimEnvironment._timeout = property(lambda self: None)

PKG, WORLD = "Competition", "CompetionMap"


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

    def record(name, ok, info):
        results.append((name, ok, info))

    with _make_env(_agent("cmd_motor_speeds", {}, [0, 0, 50])) as env:
        # 1) cmd_motor_speeds: hover thrust holds altitude with ~zero accel.
        w_hover = np.sqrt(2.0 * 9.81 / (4 * 8.54858e-6))
        d = _sensor(env.step([w_hover] * 4, 300))
        record("cmd_motor_speeds (hover z=50)",
               abs(d[8] - 50) < 0.5 and np.allclose(d[0:3], 0, atol=1e-2),
               {"z": round(d[8], 3)})

        # 2) cmd_vel
        env.scenario["agents"] = [_agent("cmd_vel", {}, [0, 0, 50])]
        env.reset()
        tgt = [1, 1, 0.5]
        vel = _sensor(env.step(tgt, 400))[3:6]
        record("cmd_vel", np.allclose(tgt, vel, atol=1e-1, rtol=1e-1),
               {"target": tgt, "vel": np.round(vel, 3).tolist()})

        # 3) cmd_vel_yaw: continuous yaw induces a horizontal limit cycle, so check
        # the mean velocity over a settling window (the meaningful tracked value).
        env.scenario["agents"] = [_agent("cmd_vel_yaw",
            {"k_v": 8, "kp_att": 12, "kd_att": 10, "kp_yaw": 0.6, "kd_yaw": 4}, [0, 0, 50], rpy=True)]
        env.reset()
        tgt = [1, 1, 0.5, 5]
        env.step(tgt, 200)
        vel = np.array([_sensor(env.step(tgt, 20), "rpy")[3:6] for _ in range(10)]).mean(axis=0)
        record("cmd_vel_yaw (mean vel)", np.allclose(tgt[:3], vel, atol=0.25, rtol=0.15),
               {"target": tgt[:3], "vel": np.round(vel, 3).tolist()})

        # 4) cmd_pos_yaw -- translation to waypoints, heading held at 0.
        for wp in ([2, 2, 60, 0], [5, -3, 55, 0]):
            env.scenario["agents"] = [_agent("cmd_pos_yaw", {}, [0, 0, 50], rpy=True)]
            env.reset()
            d = _sensor(env.step(wp, 800), "rpy")
            pos, yaw = d[6:9], float(d[-1])
            record(f"cmd_pos_yaw translate -> {wp[:3]}",
                   np.allclose(wp[:3], pos, atol=0.5, rtol=0.05) and abs(yaw) < 5,
                   {"pos": np.round(pos, 3).tolist(), "yaw": round(yaw, 2)})

        # 5) cmd_pos_yaw -- rotate in place (position held) to any heading.
        for loc, yaw_t in (([-3, 5, 58], 90), ([4, -4, 55], -90)):
            env.scenario["agents"] = [_agent("cmd_pos_yaw", {}, loc, rpy=True)]
            env.reset()
            env.step(loc + [0], 150)          # stabilize hover at heading 0
            d = _sensor(env.step(loc + [yaw_t], 900), "rpy")  # rotate in place
            pos, yaw = d[6:9], float(d[-1])
            yaw_err = abs(((yaw - yaw_t + 180) % 360) - 180)
            record(f"cmd_pos_yaw rotate-in-place -> {yaw_t}deg",
                   np.allclose(loc, pos, atol=0.5, rtol=0.05) and yaw_err < 5,
                   {"pos": np.round(pos, 3).tolist(), "yaw": round(yaw, 2)})

    print("\n===== HolybroX500 verification =====")
    allok = True
    for name, ok, info in results:
        allok &= ok
        print(f"[{'PASS' if ok else 'FAIL'}] {name:36s} {info}")
    print("=====", "ALL PASS" if allok else "SOME FAILED", "=====")
    return allok


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
