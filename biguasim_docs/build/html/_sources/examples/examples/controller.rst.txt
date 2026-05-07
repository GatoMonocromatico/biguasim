.. _`manual-control`:

=============================
Manually Controlling an Agent
=============================

We've found that `pynput` is a good library for sending keyboard commands to the agents for manual control.

Here's an example of controlling the :ref:`blue-rov-agent` using the following keyboard shortcuts.

.. list-table::
   :widths: 35 25 25
   :header-rows: 1

   * - Key
     - Forward Key
     - Backward Key
   * - Up/Down
     - i
     - k
   * - Forward/Backward
     - w
     - s
   * - Strafe Left/Right
     - a
     - d

::

    import biguasim
    import numpy as np
    from pynput import keyboard

    config = {
        "package_name": "SkyDive",
        "world": "Bridge",                            
        "main_agent": "uav0",                                                           
        "agents":[                                          
            {                                               
                "agent_name": "uav0",                       
                "agent_type": "BlueROV2",                
                "sensors": [                               
                    {
                        "sensor_type": "DynamicsSensor",
                        "socket": "IMUSocket",
                        "configuration": {
                            "UseCOM": True,
                            "UseRPY": False  
                        }
                    }
                ],                    
                "dynamics" : {
                    "batch_size" : 1,
                },                                        
                "control_abstraction": 'cmd_vel',                    
                "location" : [ -21, -70, -5], 
                "rotation": [0.0, 0.0, 0.0]               
            }
        ],
    }

    pressed_keys = set() 
    linear_speed = 3.0 *3

    def on_press(key):
        try:
            if hasattr(key, 'char'):
                pressed_keys.add(key.char)
        except AttributeError:
            pass

    def on_release(key):
        try:
            if hasattr(key, 'char'):
                pressed_keys.remove(key.char)
        except (AttributeError, KeyError):
            pass

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    def parse_keys(keys):
        cmd = np.zeros(3) 
        
        # Movimentação Linear (X, Y, Z)
        if 'w' in keys: cmd[0] += linear_speed   # Forward
        if 's' in keys: cmd[0] -= linear_speed   # Backward
        if 'a' in keys: cmd[1] += linear_speed   # Left (Strafe)
        if 'd' in keys: cmd[1] -= linear_speed   # Right (Strafe)
        if 'i' in keys: cmd[2] += linear_speed   # Up (Ascend)
        if 'k' in keys: cmd[2] -= linear_speed   # Down (Descend)

        return cmd

    with biguasim.make(scenario_cfg=config) as env:
        print("(WASD=Mov, IK=Elev, Q=Leave)")
        while True:
            if 'q' in pressed_keys:
                break
                
            command = parse_keys(pressed_keys)
            state = env.step(command.tolist())