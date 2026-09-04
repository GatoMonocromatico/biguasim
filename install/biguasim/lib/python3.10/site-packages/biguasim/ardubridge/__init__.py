"""
ardupilot_biguasim — ArduPilot JSON SITL bridge for BiguaSim.

Quick start:
    from ardupilot_biguasim import ArduBiguaSimRunner, VEHICLE_REGISTRY

    profile = VEHICLE_REGISTRY["BlueROV2"]
    scenario = ArduBiguaSimRunner.build_scenario(
        profile, package_name="Hydrone",
        world="Relief-Generic-Bridge-Vehicles-Water-Custom",
        ticks_per_sec=200,
    )
    with ArduBiguaSimRunner(profile, scenario) as runner:
        runner.run()
"""

from .bridge import ArduPilotBridge
from .frame import (
    depth_to_pressure,
    imu_glu_to_frd,
    meters_to_gps,
    pos_nwu_to_ap,
    quat_glu2nwu_to_frd2ned,
    vel_nwu_to_ned,
)
from .runner import ArduBiguaSimRunner
from .vehicle import VEHICLE_REGISTRY, VehicleProfile

__all__ = [
    "ArduPilotBridge",
    "ArduBiguaSimRunner",
    "VehicleProfile",
    "VEHICLE_REGISTRY",
    # frame utilities
    "pos_nwu_to_ap",
    "vel_nwu_to_ned",
    "imu_glu_to_frd",
    "quat_glu2nwu_to_frd2ned",
    "depth_to_pressure",
    "meters_to_gps",
]


def main() -> None:
    """Entry point: print available vehicle profiles."""
    print("ardupilot_biguasim — available vehicle profiles:")
    for name, profile in VEHICLE_REGISTRY.items():
        print(f"  {name:15s}  {profile.ardupilot_vehicle:12s}  {profile.num_motors} motors")
    print("\nSee ArduBiguaSimRunner.build_scenario() to generate a scenario config.")
