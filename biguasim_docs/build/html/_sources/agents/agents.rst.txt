.. _agents:

================
BiguaSim Agents
================

Agents are the active entities in **BiguaSim** that interact with the environment. Defined via scenario configuration files, each agent is a robotic platform equipped with specialized sensors and physics-based dynamics. 

Control is achieved by passing command arrays through the ``env.step()`` method, which are then processed by the agent's internal **Control Scheme**.

Basic Implementation
====================

To initialize a scenario with an agent, use the following configuration structure:

.. code-block:: python

   import biguasim

   scenario = {
      "name": "Harbor-Mission",
      "world": "Pier-Harbor",
      "package_name": "SkyDive",
      "agents":[
         {
            "agent_name": "uav0",
            "agent_type": "DjiMatrice",
            "location": [0, 0, 10.0],
            "control_scheme": 0,
            "sensors": [
               { "sensor_type": "RGBCamera", "socket": "CameraSocket" }
            ]
         }
      ],
   }
   
   env = biguasim.make(scenario_cfg=scenario)

   # Example command for a simple thruster/motor setup
   command = [0, 0, 0]

   for _ in range(100):
      env.step(command)


Core Fleet
==========

BiguaSim provides a diverse library of vehicles. The three primary agents, representing the most common maritime domains, are:

* :ref:`BlueROV2 <blue-rov-agent>`: A high-fidelity Remotely Operated Vehicle for underwater inspection.
* :ref:`BlueBoat <blue-boat-agent>`: A surface vessel (USV) designed for coastal surveying and logistics.
* :ref:`TorpedoAUV <torpedo-auv-agent>`: An autonomous underwater vehicle optimized for long-range surveys.

Beyond these, aerial vehicles like the :ref:`DJI Matrice <djimatrice-agent>` allow for complex multi-domain missions (Air-to-Sea).

.. toctree::
   :maxdepth: 1
   :caption: Technical Documentation

   docs/agent-config
   docs/control-abstractions


.. _agent-pages:

Available Agents Library
========================

Explore the specific capabilities, thruster configurations, and sensor mounting points for each agent.

.. note::
   **Test Agents** (Turtle and Sphere) are simplified entities used for algorithm validation and do not apply gravity or complex fluid dynamics.

Underwater Agents (AUVs/ROVs)
-----------------------------
.. toctree::
   :maxdepth: 1
   :glob:

   agents/blue-rov-agent
   agents/blue-rov-heavy-agent
   agents/torpedo-auv-agent

Water Surface Agents (USVs)
---------------------------
.. toctree::
   :maxdepth: 1
   :glob:

   agents/blue-boat-agent

Aerial Agents (UAVs)
--------------------
.. toctree::
   :maxdepth: 1
   :glob:

   agents/djimatrice-agent


Custom Development
==================
For information on integrating custom meshes or developing new vehicle dynamics in BiguaSim, see :ref:`Developing Agents <develop-agents>`.