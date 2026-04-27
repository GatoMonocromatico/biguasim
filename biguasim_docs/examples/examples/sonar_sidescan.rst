.. _visualizing_sidescan_sonar:

Visualizing Sidescan Sonar
===========================

It can be useful to visualize the output of the sonar sensor during a simulation. This script will do 
that, plotting each time sonar data is received.

Note, running this script will create octrees while running and may cause some pauses. See 
:ref:`octree` for workarounds and more info.
::

    import biguasim
    import matplotlib.pyplot as plt
    import numpy as np

    config = {
            "package_name": "SkyDive",
            "world": "Bridge",
            "main_agent" : "auv0",
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
                            "sensor_type": "SidescanSonar",
                            "socket": "SonarSocket",
                            "Hz": 10,
                            "configuration": {
                                "Azimuth": 170,
                                "Elevation": 0.25,
                                "RangeMin": 0.5,
                                "RangeMax": 40,
                                "RangeBins": 2000,
                                "AddSigma": 0.05,
                                "MultSigma": 0.05,
                                "ShowWarning": True,
                                "InitOctreeRange": 50,
                                "ViewRegion": True,
                                "ViewOctree": -10,
                                "WaterDensity": 997,
                                "WaterSpeedSound": 1480,
                                "UseApprox": True
                            }
                        }
                    ],
                    "dynamics" : {
                        "batch_size" : 1,
                    },
                    "control_abstraction": "cmd_rudders_sterns_motor_speed",
                    "location": [43, 34, -7]
                }
            ]
        }



    config_ = config['agents'][0]['sensors'][1]["configuration"]
    maxR = config_['RangeMax']
    binsR = config_['RangeBins']

    #### GET PLOT READY
    plt.ion()

    t = np.arange(0,50)
    r = np.linspace(-maxR, maxR, binsR)
    R, T = np.meshgrid(r, t)
    data = np.zeros_like(R)

    plt.grid(False)
    plot = plt.pcolormesh(R, T, data, cmap='copper', shading='auto', vmin=0, vmax=1)
    plt.tight_layout()
    plt.gca().invert_yaxis()
    plt.gcf().canvas.flush_events()

    command = [0,0,0,0,0]

    with biguasim.make(scenario_cfg=config) as env:
        for i in range(1000):
            state = env.step(command)["auv0"][0]

            if 'SidescanSonar' in state:
                data = np.roll(data, 1, axis=0)
                data[0] = state['SidescanSonar']

                plot.set_array(data.ravel())

                plt.draw()
                plt.gcf().canvas.flush_events()

    print("Finished Simulation!")
    plt.ioff()
    plt.show()