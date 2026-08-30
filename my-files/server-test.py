from biguasim.client import RemoteWorld

scenario = {"package_name": "Competition", "world": "CompetionMap"}

with RemoteWorld(address="100.119.211.18", port=8770, client_id="pilot", scenario_cfg=scenario) as world:
    world.watch_state()

    landed = world.spawn_agent(
        "uav1", "DjiMatrice",
        location=(0.0, 0.0, 5.0),
        sensors=[{"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
                  "configuration": {"UseCOM": True, "UseRPY": False}}])

    world.set_control("uav1", [330.0] * 4)
    state = world.wait_for_tick(landed + 20)