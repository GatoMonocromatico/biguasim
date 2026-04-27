.. _multi-agent_example:

Multi Agent Example
===================

BiguaSim supports simulations involving multiple vehicles at once. When multiple agents are present, 
the user needs to use the ``env.act()`` and ``env.tick()`` methods to control the agents instead of 
the ``env.step()`` method.

Press tab to switch the viewport between agents. See :ref:`hotkeys` for more.

::

    import biguasim

    cfg = {
        "name": "test_rgb_camera",
        "world": "Bridge",
        "package_name": "SkyDive",
        "main_agent": "auv0",
        "ticks_per_sec": 60,
        "agents": [
            {
                "agent_name": "auv0",
                "agent_type": "TorpedoAUV",
                "sensors": [
                    {
                        "sensor_type": "DynamicsSensor",
                        "socket": "IMUSocket",
                        "configuration": {
                            "UseCOM": True,
                            "UseRPY": False  
                        }
                    },
                    {
                        "sensor_type": "IMUSensor"
                    }
                ],
                "dynamics" : {
                        "batch_size" : 1,
                    },
                "control_abstraction": "cmd_rudders_sterns_motor_speed",
                "location": [0, 5, -5]
            },
            {
                "agent_name": "auv1",
                "agent_type": "BlueROV2",
                "sensors": [
                    {
                        "sensor_type": "DynamicsSensor",
                        "socket": "IMUSocket",
                        "configuration": {
                            "UseCOM": True,
                            "UseRPY": False  
                        }
                    },
                    {
                        "sensor_type": "DVLSensor"
                    }
                ],
                "dynamics" : {
                        "batch_size" : 1,
                    },
                "control_abstraction": "cmd_vel",
                "location": [0, 2, -5]
            }
        ]
    }

    env = biguasim.make(scenario_cfg=cfg)


    while True:
        states = env.step({'auv0': [0,0,0,0,75], "auv1" : [12, 0, 0]})

        imu = states["auv0"][0]["IMUSensor"]

        vel = states["auv1"][0]["DVLSensor"]
