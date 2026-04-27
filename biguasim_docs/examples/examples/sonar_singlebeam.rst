.. _visualizing_singlebeam_sonar:

Visualizing Singlebeam Sonar
=============================

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
                            "sensor_type": "SinglebeamSonar",
                            "socket": "SonarSocket",
                            "Hz": 10,
                            "configuration": {
                                "OpeningAngle": 30,
                                "RangeMin": 0.5,
                                "RangeMax": 30,
                                "RangeBins": 200,
                                "AddSigma": 0,
                                "MultSigma": 0,
                                "RangeSigma": 0.1,
                                "ShowWarning": False,
                                "InitOctreeRange": 40,
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
                    "location": [36, 28, -1]
                }
            ]
        }

    #### GET SONAR CONFIG
    config_ = config['agents'][0]['sensors'][-1]["configuration"]
    minR = config_['RangeMin']
    maxR = config_['RangeMax']
    binsR = config_['RangeBins']

    #### GET PLOT READY
    plt.ion()

    t = np.arange(0,50)
    r = np.linspace(minR, maxR, binsR)
    T, R = np.meshgrid(t, r)
    data = np.zeros_like(R)

    plt.grid(False)
    plot = plt.pcolormesh(T, R, data, cmap='gray', shading='auto', vmin=0, vmax=1)
    plt.tight_layout()
    plt.gca().invert_yaxis()
    plt.gcf().canvas.flush_events()

    #### RUN SIMULATION
    command = [0,0,0,0,20]
    with biguasim.make(scenario_cfg=config) as env:
        for i in range(1000):
            state = env.step(command)["auv0"][0]

            if 'SinglebeamSonar' in state:
                data = np.roll(data, 1, axis=1)
                data[:,0] = state['SinglebeamSonar']

                plot.set_array(data.ravel())

                plt.draw()
                plt.gcf().canvas.flush_events()

    print("Finished Simulation!")
    plt.ioff()
    plt.show()