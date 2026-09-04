"""Verification for the KopisX8 competition drone.

Holybro Kopis X8 Cinelifter 5" (Caged), modelled as a quad on the four top
rotors. Runs in two phases:

  MODEL  -- pure Python, no simulator. Checks the physical parameters against
            the manufacturer's bench figures and the control allocation.
  ENGINE -- drives every control abstraction in the Competition/CompetionMap
            build and checks the drone reaches the expected state.

The engine phase needs the KopisX8 blueprint registered in the engine's
AgentBpMap; until that exists, run the model phase alone:

    python tests/test_kopisx8.py --model-only

With the engine ready, run both (biguasim conda env):

    python tests/test_kopisx8.py

Operational note (cmd_pos_yaw)
------------------------------
Translation and large rotation are done as SEPARATE phases, each from a
stabilized state, same as the HolybroX500: translate holding heading 0, then
rotate in place. Yawing while translating couples the loops.
"""
import sys
import uuid

import numpy as np
import torch

import biguasim
from biguasim.agents import KopisX8 as KopisX8Agent, AgentDefinition
from biguasim.dynamics.agents import ModelsFactory
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY

# First-tick shader compilation can exceed the hardcoded 60s timeout.
biguasim.environments.BiguaSimEnvironment._timeout = property(lambda self: None)

PKG, WORLD = "Competition", "CompetionMap"
AGENT = "KopisX8"

G = 9.80665
# T-Motor bench figures for the Velox V2207 V2 1750KV on 6S, which k_eta and
# k_m are anchored on. See biguasim.dynamics.agents.KopisX8.
BENCH_THRUST_G = 1681.0   # g per motor at full throttle
BENCH_POWER_W = 816.0     # electrical input at that point

results = []


def record(name, ok, info):
    results.append((name, ok, info))
    return ok


# ---------------------------------------------------------------------------
# MODEL phase -- no simulator needed
# ---------------------------------------------------------------------------

def model_phase():
    model = ModelsFactory.build_model(AGENT)
    p = model._params

    record("registry: AgentDefinition resolves",
           AgentDefinition("uav0", AGENT).type is KopisX8Agent,
           {"class": AgentDefinition("uav0", AGENT).type.__name__})

    w_hover = float(np.sqrt(p["mass"] * G / (4 * p["k_eta"])))
    thrust_1 = p["k_eta"] * p["rotor_speed_max"] ** 2
    thrust_4 = 4 * thrust_1

    # k_eta must reproduce the motor's published max thrust at the rotor cap.
    grams = thrust_1 / G * 1000
    record("k_eta matches T-Motor bench thrust",
           abs(grams - BENCH_THRUST_G) / BENCH_THRUST_G < 0.02,
           {"model_g": round(grams), "bench_g": BENCH_THRUST_G})

    # k_m must put shaft power at a believable motor+ESC efficiency. This is
    # the independent check: thrust alone cannot pin down k_m.
    eff = p["k_m"] * p["rotor_speed_max"] ** 3 / BENCH_POWER_W
    record("k_m gives plausible motor+ESC efficiency",
           0.70 <= eff <= 0.80,
           {"shaft_W": round(p["k_m"] * p["rotor_speed_max"] ** 3), "eff": round(eff, 3)})

    # It has to be able to lift, and hover has to sit inside the rotor envelope.
    record("thrust-to-weight > 2",
           thrust_4 / (p["mass"] * G) > 2.0,
           {"T/W": round(thrust_4 / (p["mass"] * G), 2)})
    record("hover inside rotor envelope",
           0 < w_hover < p["rotor_speed_max"],
           {"hover": round(w_hover, 1), "max": p["rotor_speed_max"],
            "pct": round(100 * w_hover / p["rotor_speed_max"], 1)})

    # The rotor cap appears in three places; a mismatch silently limits thrust
    # (this is exactly what grounded the HolybroX500).
    stub = type("S", (), {"_current_control_scheme": 0})()
    space_max = [sp for n, sp in KopisX8Agent.control_abstractions.fget(stub)
                 if n.startswith("[r1")][0].get_high()[0]
    profile = VEHICLE_REGISTRY[AGENT]
    pwm_max = profile.pwm_converters[0](2000.0)
    record("rotor cap agrees across the three definitions",
           space_max == pwm_max == p["rotor_speed_max"],
           {"action_space": space_max, "pwm@2000": pwm_max, "params": p["rotor_speed_max"]})

    record("ardubridge profile is self-consistent",
           profile.num_motors == len(p["rotor_pos"]) == len(profile.motor_mapping)
           == len(profile.pwm_converters) == len(profile.motor_signs)
           and sorted(profile.motor_mapping) == [0, 1, 2, 3],
           {"motors": profile.num_motors, "mapping": profile.motor_mapping})

    # Control allocation: pure weight, zero moment -> four identical rotors at
    # hover. Verified for every abstraction, since each builds TM_to_f.
    ok = True
    for ca in ("cmd_motor_speeds", "cmd_vel", "cmd_vel_yaw", "cmd_pos_yaw"):
        m = model(batch_size=1, device="cpu", control_abstraction=ca)
        TM = torch.tensor([[p["mass"] * G, 0.0, 0.0, 0.0]], dtype=torch.float64)
        f = torch.einsum("bij,bj->bi", m.batched_params.TM_to_f, TM)
        w = torch.sqrt(f / m.batched_params.k_eta)
        ok &= bool(m.batched_params.num_rotors == 4
                   and torch.allclose(w, torch.full_like(w, w_hover), rtol=1e-12))
    record("allocation: weight/no moment -> 4 equal rotors at hover", ok,
           {"rotors": 4, "abstractions": 4})

    def state(v=(0, 0, 0), x=(0, 0, 50), w=(0, 0, 0), q=(0, 0, 0, 1)):
        d = np.zeros(19)
        d[3:6], d[6:9], d[12:15], d[15:19] = v, x, w, q
        return [{"DynamicsSensor": d.tolist()}]

    m = model(batch_size=1, device="cpu", control_abstraction="cmd_motor_speeds")
    # Tolerance is 1e-5, not 0: base_model builds the command tensor with
    # torch.as_tensor(...).double(), which infers float32 first, so the rotor
    # command loses ~1e-7 of precision before reaching the model.
    a = np.array(m.step(state(), [[w_hover] * 4], 1 / 30))[0]
    record("step at hover -> zero acceleration",
           np.allclose(a, 0, atol=1e-5),
           {"accel": np.round(a, 7).tolist()})

    a = np.array(m.step(state(), [[p["rotor_speed_max"]] * 4], 1 / 30))[0]
    expected = G * (thrust_4 / (p["mass"] * G) - 1)
    record("step at full thrust -> g*(T/W - 1)",
           abs(a[2] - expected) < 1e-3,
           {"az": round(a[2], 3), "expected": round(expected, 3)})

    m = model(batch_size=1, device="cpu", control_abstraction="cmd_vel")
    s = {"x": torch.tensor([[0.0, 0.0, 50.0]]).double(), "v": torch.zeros(1, 3).double(),
         "q": torch.tensor([[0.0, 0.0, 0.0, 1.0]]).double(), "w": torch.zeros(1, 3).double()}
    wc = m.get_cmd_motor_speeds(s, {"cmd_ctrl": torch.zeros(1, 3).double()})
    # rtol 1e-7: uav.py builds the gravity vector as torch.tensor([0, 0, g]),
    # which is float32 before the .double() cast. Same slip as above, and it
    # affects the HolybroX500 and DjiMatrice identically.
    record("cmd_vel(0,0,0) at rest -> hover speeds",
           bool(torch.allclose(wc, torch.full_like(wc, w_hover), rtol=1e-7)),
           {"cmd": round(float(wc[0, 0]), 1), "hover": round(w_hover, 1)})

    return w_hover, p


# ---------------------------------------------------------------------------
# ENGINE phase -- needs the KopisX8 blueprint in the packaged build
# ---------------------------------------------------------------------------

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
        "agent_name": "uav0", "agent_type": AGENT, "sensors": sensors,
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


def engine_phase(w_hover):
    with _make_env(_agent("cmd_motor_speeds", {}, [0, 0, 50])) as env:
        # 1) cmd_motor_speeds: hover thrust holds altitude with ~zero accel.
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

        # 3) cmd_vel_yaw: continuous yaw induces a horizontal limit cycle, so
        # check the mean velocity over a settling window. Run on the model's
        # own gains -- if this one needs a retune, that is what it should say.
        env.scenario["agents"] = [_agent("cmd_vel_yaw", {}, [0, 0, 50], rpy=True)]
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
            env.step(loc + [0], 150)                          # stabilize at heading 0
            d = _sensor(env.step(loc + [yaw_t], 900), "rpy")  # rotate in place
            pos, yaw = d[6:9], float(d[-1])
            yaw_err = abs(((yaw - yaw_t + 180) % 360) - 180)
            record(f"cmd_pos_yaw rotate-in-place -> {yaw_t}deg",
                   np.allclose(loc, pos, atol=0.5, rtol=0.05) and yaw_err < 5,
                   {"pos": np.round(pos, 3).tolist(), "yaw": round(yaw, 2)})


def main():
    model_only = "--model-only" in sys.argv

    print("\n===== KopisX8 verification: MODEL =====")
    w_hover, p = model_phase()
    for name, ok, info in results:
        print(f"[{'PASS' if ok else 'FAIL'}] {name:46s} {info}")

    if not model_only:
        print("\n===== KopisX8 verification: ENGINE =====")
        before = len(results)
        try:
            engine_phase(w_hover)
        except Exception as exc:
            record("engine phase reached the simulator", False, {"error": repr(exc)[:160]})
            print("  (needs the KopisX8 blueprint in the AgentBpMap of the packaged "
                  "build; run with --model-only to skip)")
        for name, ok, info in results[before:]:
            print(f"[{'PASS' if ok else 'FAIL'}] {name:46s} {info}")

    allok = all(ok for _, ok, _ in results)
    n_pass = sum(1 for _, ok, _ in results if ok)
    print(f"\n===== {n_pass}/{len(results)} PASS -- "
          f"{'ALL PASS' if allok else 'SOME FAILED'} =====")
    return allok


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
