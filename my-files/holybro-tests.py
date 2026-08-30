"""
BlueROV2 + ArduPilot SITL, with the depth camera read out of the state dict.

This is ArduBiguaSimRunner.run() unrolled: we own the env and the bridge, so the
per-tick agent_state (and therefore the camera images) is in scope.

Start ArduPilot SITL first (it drives the clock — the sim stays frozen until it
sends its first servo packet):

    sim_vehicle.py -v ArduSub -L RATBeach --console --map -f JSON:127.0.0.1
"""

import numpy as np

import biguasim
from biguasim.ardubridge import ArduBiguaSimRunner, ArduPilotBridge, VEHICLE_REGISTRY

PROFILE = VEHICLE_REGISTRY["HolybroX500"]   # or BlueROVHeavy, DjiMatrice, TorpedoAUV, BlueBoat
TICKS_PER_SEC = 200

scenario = ArduBiguaSimRunner.build_scenario(
    PROFILE,
    package_name="Competition",
    world="CompetionMap",
    location=[0, 0, 1],
    rotation=[0.0, 0.0, 0.0],   # [roll, pitch, yaw] in degrees
    ticks_per_sec=TICKS_PER_SEC,
)

# build_scenario only adds the sensors the bridge needs, so append the camera.
# Width/Height and CaptureWidth/CaptureHeight are both set on purpose: the UE
# side reads the former, sensors.py:363-370 sizes the shared-memory buffer from
# the latter, and a mismatch gives you a wrongly-shaped array.
# scenario["agents"][0]["sensors"].append(
#     {
#         "sensor_type": "DepthCamera",
#         "sensor_name": DEPTH_NAME,
#         "socket": "CameraSocket",
#         "Hz": DEPTH_HZ,
#         "configuration": {
#             "CaptureWidth": DEPTH_RES,
#             "CaptureHeight": DEPTH_RES,
#             "Width": DEPTH_RES,
#             "Height": DEPTH_RES,
#             "FOV": 90.0,
#             "MinDistance": 0.1,
#             "MaxDistance": DEPTH_MAX_M,
#             "ShowDisplay": False,
#         },
#     }
# )

AGENT = scenario["main_agent"]
DT = 1.0 / TICKS_PER_SEC
# DISPLAY_EVERY = TICKS_PER_SEC // DEPTH_HZ


env = biguasim.make(scenario_cfg=scenario, show_viewport=True, verbose=False)
bridge = ArduPilotBridge(PROFILE, address="127.0.0.1", port=9002)

with env:
    bridge.bind()
    motor_cmds = [0.0] * PROFILE.num_motors
    raw = env.step(motor_cmds)          # prime the state; make() already reset()
    sim_time = 0.0
    tick = 0
    last_good_state = None

    print(f"Running {PROFILE.name} SITL bridge (Ctrl-C to stop)...")
    try:
        while True:
            frame, pwm = bridge.receive_pwm()
            if frame is None:
                continue                # ArduPilot is the clock: no packet, no tick

            motor_cmds = bridge.pwm_to_motor_cmds(pwm, frame)
            raw = env.step(motor_cmds)
            agent_state = raw[AGENT][0]
            sim_time += DT
            tick += 1

            json_state = bridge.build_json_state(agent_state, sim_time)
            if json_state is None and last_good_state is not None:
                # a non-finite sensor value this tick; resend the last good pose
                # with a fresh timestamp so SITL's clock keeps advancing
                json_state = dict(last_good_state, timestamp=sim_time)
            elif json_state is not None:
                last_good_state = json_state
            if json_state is not None:
                bridge.send_state(json_state)
    except KeyboardInterrupt:
        print("Bridge stopped.")
    finally:
        bridge.close()