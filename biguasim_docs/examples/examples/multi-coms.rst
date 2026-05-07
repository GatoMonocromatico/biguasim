.. _multi-agent-comms_example:

============================
Multi-Agent Communications
============================

Many times it's necessary to communicate between agents. This can be done using the 
``AcousticBeaconSensor`` or the ``OpticalModemSensor``. Below are some examples of these.

Sending Acoustic Messages
=========================

The command :meth:`biguasim.environments.BiguaSimEnvironment.send_acoustic_message` is used to send messages between acoustic beacons.
There's a number of message types that can be sent, all with varying functionality, see :class:`biguasim.sensors.AcousticBeaconSensor`
for details.

Further, a few helper functions exist if needed, 

- :py:attr:`biguasim.environments.BiguaSimEnvironment.beacons` returns all beacons.
- :py:attr:`biguasim.environments.BiguaSimEnvironment.beacons_id` returns all beacons' ids.
- :py:attr:`biguasim.environments.BiguaSimEnvironment.beacons_status` returns all beacons' status (whether it's transmitting or not).

::

    import biguasim

    cfg = {
        "name": "test_acou_coms",
        "world": "Pier-Harbor",
        "package_name": "SkyDive",
        "main_agent": "auv0",
        "ticks_per_sec": 200,
        "agents": [
            {
                "agent_name": "auv0",
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
                        "sensor_type": "AcousticBeaconSensor",
                        "location": [0,0,0],
                        "configuration": {
                            "id": 0
                        }
                    },
                ],
                "dynamics" : { 
                    "batch_size" : 1
                    },
                "control_abstraction": "cmd_vel",
                "location": [0, 0, -5]
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
                        "sensor_type": "AcousticBeaconSensor",
                        "location": [0,0,0],
                        "configuration": {
                            "id": 1
                        }
                    },
                ],
                "dynamics" : { 
                    "batch_size" : 1
                    },
                "control_abstraction": "cmd_vel",
                "location": [0, 100, -5]
            }
        ]
    }

    env = biguasim.make(scenario_cfg=cfg)
    env.reset()

    # This is how you send a message from one acoustic com to another
    # This sends from id 0 to id 1 (ids configured above)
    # with message type "OWAY" and data "my_data_payload"
    env.send_acoustic_message(0, 1, "OWAY", "my_data_payload")

    for i in range(500):
        states = env.tick()

        if "AcousticBeaconSensor" in states['auv1'][0]:
            # For this message, should receive back [message_type, from_sensor, data_payload]
            print(i, "Received:", states['auv1'][0]["AcousticBeaconSensor"])
            break
        else:
            print(i, "No message received")


Sending Optical Messages
========================

The command :meth:`biguasim.environments.BiguaSimEnvironment.send_optical_message` is used to send messages between optical modems.
See :class:`biguasim.sensors.OpticalModemSensor` for configuration details. Note in order for a message to be transmitted,
the 2 sensors must be aligned.

Further, a few helper functions exist if needed, 

- :py:attr:`biguasim.environments.BiguaSimEnvironment.modems` returns all modems.
- :py:attr:`biguasim.environments.BiguaSimEnvironment.modems_id` returns all modems' ids.

::

    import biguasim

    cfg = {
        "name": "test_acou_coms",
        "world": "Pier-Harbor",
        "package_name": "SkyDive",
        "main_agent": "auv0",
        "ticks_per_sec": 200,
        "agents": [
            {
                "agent_name": "auv0",
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
                        "sensor_type": "OpticalModemSensor",
                        "location": [0,0,0],
                        "socket": "COM",
                        "configuration": {
                            "id": 0
                        }
                    },
                ],
                "dynamics" : { 
                    "batch_size" : 1
                    },
                "control_abstraction": "cmd_vel",
                "location": [0, 0, -5]
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
                        "sensor_type": "OpticalModemSensor",
                        "location": [0,0,0],
                        "socket": "COM",
                        "configuration": {
                            "id": 1
                        }
                    },
                ],
                "dynamics" : { 
                    "batch_size" : 1
                    },
                "control_abstraction": "cmd_vel",
                "location": [0, 100, -5]
            }
        ]
    }

    env = biguasim.make(scenario_cfg=cfg)
    env.reset()

    # This is how you send a message from one optical com to another
    # This sends from id 0 to id 1 (ids configured above)
    # with data "my_data_payload"
    env.send_optical_message(0, 1, "my_data_payload")

    for i in range(500):
        states = env.tick()

        if "OpticalModemSensor" in states['auv1'][0]:
            print(i, "Received:", states['auv1'][0]["OpticalModemSensor"])
            break
        else:
            print(i, "No message received")