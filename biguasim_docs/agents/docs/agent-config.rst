.. _`agent-configuration`:

===================
Agent Configuration
===================

BiguaSim agents are declared in a list in the scenario dictionary. Each agent is defined using a
new dictionary in the list.

.. note::
   The first agent in the ``agents`` array is the "main agent".

.. code-block:: json

   "agents":[
      {
         "agent_name": "uav0",
         "agent_type": "{agent types}",
         "sensors": [
            "array of sensor objects"
         ],
         "location": [1.0, 2.0, 3.0],
         "rotation": [1.0, 2.0, 3.0],
         "location_randomization": [1, 2, 3],
         "rotation_randomization": [10, 10, 10],
         "control_abstraction": "{control abstraction type}"
         "dynamics" : {
                "batch_size" : 1,
         }, 
      },
      { 
         "agent_name": "uav1"
         ...
      }
   ]

Below is a description of the keys in the agent dictionary:


Agent Name
==========
``agent_name`` is a string that specifies the name of the agent. This name is used to identify the 
agent when interacting with the environment.


Agent Type
==========
``agent_type`` is a string that specifies the type of agent. This table gives the current valid 
arguments for ``agent_type``:

=========================== ========================
Agent Type                  String in agent_type
=========================== ========================
:ref:`blue-boat-agent`      ``BlueBoat``
:ref:`torpedo-auv-agent`    ``TorpedoAUV``
:ref:`blue-rov-agent`       ``BlueROV2``
:ref:`blue-rov-heavy-agent` ``BlueROV2 Heavy``
:ref:`djimatrice-agent`     ``DjiMatrice``
=========================== ========================

Details on each agent can be found at the individual agent pages in :ref:`agents`. This list will 
be updated as more agents are added to BiguaSim.


Sensors
=======
``sensors`` is an array of sensor objects that are attached to the agent. Each sensor is defined 
using a new dictionary in the list. For details on configuring and using sensors, see 
:ref:`sensor-configuration`.


Location and Rotation
=====================
These keys define the location and orientation of the agent in the world, measured in meters and 
degrees respectively. The location is in the format ``[dx, dy, dz]`` and the rotation is 
``[roll, pitch, yaw]``, rotated about XYZ fixed axes, ie R_z R_y R_x.

.. note::
   - Location uses BiguaSim world coordinates, not Unreal Engine level coordinates! 
   - BiguaSim coordinates are **right handed** in meters. 
   - See :ref:`units-and-coordinates` for details. 


.. _`location-randomization`:

Location Randomization
======================
``location_randomization`` and ``rotation_randomization`` are optional. If provided, the agent's 
start location and/or rotation will vary by a random amount sampled uniformly from the specified 
range. Randomization in each direction is sampled independently. 

The location randomization value is measured in meters, in the format ``[dx, dy, dz]``. The rotation 
randomization is in the format ``[roll, pitch, yaw]``, rotated about the XYZ fixed axes (i.e. 
R_z*R_y*R_x).


Control Abstraction
===================

The ``control_abstraction`` string defines how the simulator interprets commands sent to an agent via the ``step`` or ``tick`` methods. It acts as the interface layer between high-level autonomous logic and low-level physics.

Standardized Agents
-------------------

In BiguaSim, most agents (such as the **BlueROV2** and **HoveringAUV**) share a unified set of control abstractions. These allow for seamless switching between different levels of autonomy:

* **``cmd_vel``**: Controls linear velocity :math:`(v_x, v_y, v_z)`.
* **``cmd_vel_yaw``**: Similar to ``cmd_vel``, but maintains a specific target yaw heading during maneuvers.
* **``cmd_pos_yaw``**: A high-level abstraction where the user provides target coordinates :math:`[x, y, z]` and a target yaw. An internal PID controller handles the trajectory.
* **``accel``**: Provides direct acceleration commands along the agent's axes.
* **``cmd_motor_speeds``**: The lowest level of control for standard agents, allowing users to send individual thrust values to each motor (e.g., 8 independent thrusters for the BlueROV2).

The TorpedoAUV Exception
------------------------

The **TorpedoAUV** does not follow the standard multi-thruster abstractions due to its unique underactuated physical design (single rear propeller and control surfaces). It uses a specialized abstraction:

* **``cmd_rudders_sterns_motor_speed``**: Requires a specific vector to control the main motor RPM and the deflection angles of the rudders and stern planes.
    * **Input Vector**: ``[Motor_RPM, Rudder1, Rudder2, Stern1, Stern2]``

For more details on specific control implementation and PID tuning, see :ref:`control-abstractions`.
