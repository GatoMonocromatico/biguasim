===============
Getting Started
===============

First, see :ref:`installation` to get the ``biguasim`` package and 
``SkyDive`` installed.

A minimal BiguaSim usage example is below:

::

   import biguasim

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
            ],                    
            "dynamics" : {
                "batch_size" : 1,
            },                                        
            "control_abstraction": 'cmd_vel',                    
            "location" : [ -150, -125, 50], 
            "rotation": [0.0, 40.0, 0]               
        }
      ],
   }

   env = biguasim.make(scenario_cfg = config)

   # The agent in cmd_vel configuration takes a command for each cartesian position (X,Y,Z)
   command = [15, 0, 0]

   for _ in range(2000):
      state = env.step(command)
   print("Finished!")


Notice that:

1. You pass the name of a :ref:`scenario<scenarios>` into ``biguasim.make``. The scenario 
defines a world, the agents in it, the sensors on the agents, and so on. ``biguasim.make``
returns an environment object that you can interact with.
   
   See :ref:`all-packages` for all of the different worlds and pre-built scenarios that
   are available, or make your own custom scenario.

2. You pass a command to the environment object with ``env.step``. The command is passed to the 
agent in the world, and the environment is updated. The environment returns a dictionary of the 
state of the world after the update, including sensor data. 

   You can access data from a specific sensor with the state dictionary:
   ::

      dvl = state["DVLSensor"]

**That's it!** BiguaSim is meant to be fairly simple to use.

Check out the different :ref:`worlds<all-packages>` that are available, read the
:ref:`API documentation<biguasim-api-index>`, or get started on making your own
custom :ref:`scenarios<scenarios>`.
