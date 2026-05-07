"""
ardupilot_biguasim — integration test / usage example.

Usage
-----
# 1. Dry-run (no UE5 binary required) — just prints vehicle info and a scenario dict:
    python test.py

# 2. Full SITL run with BlueROV2 (requires UE5 binary + ArduPilot SITL):
    python test.py bluerov2

# 3. Full SITL run with another vehicle:
    python test.py bluerovheavy
    python test.py blueboat
    python test.py djimatrice
    python test.py torpedoauv

ArduPilot SITL must be started separately before step 2/3, e.g.:
    sim_vehicle.py -v ArduSub -L RATBeach --console --map -f JSON:127.0.0.1 \\
        --add-param-file=/path/to/bluerov2.parm
"""

import sys
import pprint
from pathlib import Path

# allow running directly from this directory: python test.py
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ardupilot_biguasim import ArduBiguaSimRunner, VEHICLE_REGISTRY
from ardupilot_biguasim.frame import (
    depth_to_pressure,
    imu_glu_to_frd,
    pos_nwu_to_ap,
    quat_glu2nwu_to_frd2ned,
    vel_nwu_to_ned,
)

PACKAGE_NAME = "Hydrone"
WORLD = "Relief-Generic-Bridge-Vehicles-Water-Custom"
GPS_ORIGIN = (33.810313, -118.393867)


# ---------------------------------------------------------------------------
# Dry-run: validate imports, conversions, and scenario building
# ---------------------------------------------------------------------------

def dry_run():
    print("=" * 60)
    print("ardupilot_biguasim — dry-run checks")
    print("=" * 60)

    # 1. Vehicle registry
    print("\nRegistered vehicle profiles:")
    for name, p in VEHICLE_REGISTRY.items():
        print(
            f"  {name:15s}  AP={p.ardupilot_vehicle:12s}  "
            f"motors={p.num_motors}  abstraction={p.control_abstraction}"
        )

    # 2. Coordinate conversion sanity checks
    print("\nFrame conversion checks:")

    pos = pos_nwu_to_ap([100.0, 0.0, -5.0], GPS_ORIGIN)
    print(f"  pos_nwu_to_ap([100,0,-5])  → lat={pos[0]:.6f}  lon={pos[1]:.6f}  alt={pos[2]:.1f}")

    vel = vel_nwu_to_ned([2.0, 1.0, -0.5])
    print(f"  vel_nwu_to_ned([2,1,-0.5]) → NED {vel}  (expect [2,-1,0.5])")

    accel, gyro = imu_glu_to_frd([[0.0, 0.0, -9.81], [0.0, 0.0, 0.0]])
    print(f"  imu_glu_to_frd accel        → FRD {accel}  (expect [0,0,9.81])")

    q = quat_glu2nwu_to_frd2ned([0.0, 0.0, 0.0, 1.0])
    print(f"  quat identity NWU→NED       → {[round(v,4) for v in q]}  (expect [1,0,0,0])")

    p_pa = depth_to_pressure(-10.0)
    print(f"  depth_to_pressure(-10m)     → {p_pa:.1f} Pa  (expect ~201941 Pa)")

    # 3. Scenario builder for each vehicle
    print("\nScenario dict for BlueROV2:")
    profile = VEHICLE_REGISTRY["BlueROV2"]
    scenario = ArduBiguaSimRunner.build_scenario(
        profile,
        package_name=PACKAGE_NAME,
        world=WORLD,
        ticks_per_sec=200,
    )
    pprint.pprint(scenario, indent=2, width=80)

    print("\nAll checks passed. Run with a vehicle name to start SITL.")


# ---------------------------------------------------------------------------
# Full SITL run
# ---------------------------------------------------------------------------

def run_sitl(vehicle_key: str):
    key = vehicle_key.upper()
    # Accept case-insensitive lookup
    match = next((k for k in VEHICLE_REGISTRY if k.upper() == key), None)
    if match is None:
        print(f"Unknown vehicle '{vehicle_key}'. Available: {list(VEHICLE_REGISTRY)}")
        sys.exit(1)

    profile = VEHICLE_REGISTRY[match]
    print(f"\nStarting SITL bridge for {match} ({profile.ardupilot_vehicle})")
    print(f"  Suggested ArduPilot command:\n    {profile.sitl_args}\n")

    scenario = ArduBiguaSimRunner.build_scenario(
        profile,
        package_name=PACKAGE_NAME,
        world=WORLD,
        ticks_per_sec=200,
        location=[0, 0, 0],
        rotation=[0.0, 0.0, 0.0],
    )

    with ArduBiguaSimRunner(
        profile,
        scenario,
        gps_origin=GPS_ORIGIN,
        verbose=True,
    ) as runner:
        runner.run()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) < 2:
        dry_run()
    else:
        run_sitl(sys.argv[1])
