"""
ArduBiguaSimRunner — wires a BiguaSim environment to an ArduPilotBridge.

Usage:
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

from __future__ import annotations

import biguasim

from .bridge import ArduPilotBridge
from .vehicle import VehicleProfile

_DEFAULT_GPS_ORIGIN = (33.810313, -118.393867)


class ArduBiguaSimRunner:
    """Context-manager that owns both the BiguaSim environment and the ArduPilot bridge."""

    def __init__(
        self,
        profile: VehicleProfile,
        scenario: dict,
        address: str = "127.0.0.1",
        port: int = 9002,
        gps_origin: tuple = _DEFAULT_GPS_ORIGIN,
        show_viewport: bool = False,
        verbose: bool = False,
    ):
        self._profile = profile
        self._scenario = scenario
        self._verbose = verbose
        self._show_viewport = show_viewport

        self._env = biguasim.make(scenario_cfg=scenario, show_viewport=show_viewport, verbose=verbose)
        self._bridge = ArduPilotBridge(profile, address=address, port=port, gps_origin=gps_origin)
        self._dt = 1.0 / scenario.get("ticks_per_sec", 200)
        self._agent_name = scenario.get("main_agent", profile.name.lower() + "0")

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Blocking main loop. Returns only on KeyboardInterrupt."""
        bridge = self._bridge
        env = self._env
        agent = self._agent_name
        dt = self._dt

        bridge.bind()
        motor_cmds = [0.0] * self._profile.num_motors
        raw = env.step(motor_cmds)
        agent_state = raw[agent][0]
        sim_time = 0.0

        print(f"Running {self._profile.name} SITL bridge (Ctrl-C to stop)...")
        try:
            while True:
                frame, pwm = bridge.receive_pwm()
                if frame is None:
                    continue
                motor_cmds = bridge.pwm_to_motor_cmds(pwm, frame)

                raw = env.step(motor_cmds)
                agent_state = raw[agent][0]
                sim_time += dt

                json_state = bridge.build_json_state(agent_state, sim_time)
                bridge.send_state(json_state)

                if self._verbose and frame is not None and frame % 200 == 0:
                    print(
                        f"  t={sim_time:.2f}s frame={frame} "
                        f"quat={[f'{v:.3f}' for v in json_state['quaternion']]} "
                        f"motors={[f'{m:.1f}' for m in motor_cmds]}"
                    )
        except KeyboardInterrupt:
            print("Bridge stopped.")
        finally:
            bridge.close()

    # ------------------------------------------------------------------
    # Scenario builder
    # ------------------------------------------------------------------

    @staticmethod
    def build_scenario(
        profile: VehicleProfile,
        package_name: str,
        world: str,
        agent_name: str = "",
        location: list | None = None,
        rotation: list | None = None,
        ticks_per_sec: int = 200,
        **overrides,
    ) -> dict:
        """
        Build a minimal BiguaSim scenario dict with all sensors required for SITL.

        Extra keyword args are merged into the top-level scenario dict.
        To override agent fields, pass them as overrides prefixed with 'agent_':
          e.g. agent_location=[10, 0, 0].
        """
        if not agent_name:
            agent_name = profile.name.lower() + "0"

        sensors = [
            {
                "sensor_type": "DynamicsSensor",
                "socket": "COM",
                "configuration": {"UseCOM": True, "UseRPY": False},
            },
            {
                "sensor_type": "LocationSensor",
                "socket": "COM",
                "configuration": {"Sigma": 0},
            },
            {"sensor_type": "VelocitySensor", "socket": "COM"},
            {
                "sensor_type": "IMUSensor",
                "socket": "IMUSocket",
                "Hz": ticks_per_sec,
                "configuration": {
                    "AccelSigma": 0.0,
                    "AngVelSigma": 0.0,
                    "AccelBiasSigma": 0.0,
                    "AngVelBiasSigma": 0.0,
                    "ReturnBias": False,
                },
            },
        ]

        if profile.include_depth_sensor:
            sensors.append({
                "sensor_type": "DepthSensor",
                "socket": "DepthSocket",
                "Hz": ticks_per_sec,
                "configuration": {"Sigma": 0.0},
            })

        agent_cfg = {
            "agent_name": agent_name,
            "agent_type": profile.name,
            "sensors": sensors,
            "dynamics": {"batch_size": 1},
            "control_abstraction": profile.control_abstraction,
            "location": location or [0, 0, 0],
            "rotation": rotation or [0.0, 0.0, 0.0],
        }

        # Allow per-agent overrides via agent_ prefix
        for key in list(overrides.keys()):
            if key.startswith("agent_"):
                agent_cfg[key[len("agent_"):]] = overrides.pop(key)

        scenario = {
            "package_name": package_name,
            "world": world,
            "main_agent": agent_name,
            "ticks_per_sec": ticks_per_sec,
            "frames_per_sec": False,
            "octree_min": 0.02,
            "octree_max": 5.0,
            "agents": [agent_cfg],
        }
        scenario.update(overrides)
        return scenario

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "ArduBiguaSimRunner":
        self._env.__enter__()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self._bridge.close()
        self._env.__exit__(exc_type, exc_val, exc_tb)
