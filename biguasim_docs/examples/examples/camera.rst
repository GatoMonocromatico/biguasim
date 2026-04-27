.. _visualizing_rgbcamera_output:

Visualizing RGBCamera Output
============================

It can be useful to display the output of the RGB camera while an agent is 
training. Below is an example using the ``cv2`` library.

When the window is open, press the ``0`` key to tick the environment and show the
next window.

::

    import biguasim, cv2
    import numpy as np

    config = {
        "package_name": "SkyDive",
        "world": "Pier-Harbor",                            
        "main_agent": "uav0",                                                           
        "agents":[                                          
            {                                               
                "agent_name": "uav0",                       
                "agent_type": "DjiMatrice",                
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
                        "sensor_type": "RGBCamera",
                        "sensor_name": "RGBCamera",
                        "socket": "CameraSocket",
                        "Hz": 5,
                        "configuration": {
                            "CaptureWidth": 512,
                            "CaptureHeight": 512
                        }
                    }
                ],                    
                "dynamics" : {
                    "batch_size" : 1,
                },                                        
                "control_abstraction": 'cmd_vel',                    
                "location" : [ -21, -136, 15], 
                "rotation": [0.0, 0.0, 0.0]               
            }
        ],
    }

    env = biguasim.make(scenario_cfg = config)
    command = [0, 0, 10]

    for _ in range(200):
        state = env.step(command)["uav0"][0]
        if "RGBCamera" in state:
            pixels = state["RGBCamera"]
            frame = pixels[:, :, 0:3].astype(np.uint8)
            
            cv2.imshow("Camera Output", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()