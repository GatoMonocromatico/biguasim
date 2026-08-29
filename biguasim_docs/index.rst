Welcome to BiguaSim's documentation!
==============================================
.. image:: _static/images/logo-light.svg
   :alt: BiguaSim hero
   :class: main-logo

BiguaSim is a high-fidelity simulator developed by `Nautec <https://nautec.furg.br/>`_
at `Federal University of Rio Grande <https://www.furg.br>`_ .

Features
""""""""
#. Multi-Domain Simulation Environments: Rich, high-fidelity worlds supporting simultaneous operations of both air and water vehicles (UAVs and AUVs).
#. Powered by Unreal Engine 5 (UE5): Leverages modern UE5 rendering and physics for maximum visual realism and high-detail environments.
#. Synthetic Dataset Generation: Purpose-built infrastructure for generating realistic, annotated synthetic data to train perception and AI models.
#. Comprehensive Sensor Suite: Complete with common multi-domain sensors including DVL, IMU, optical cameras, high-precision depth cameras, and more, customized for both aerial and underwater domains.
#. Advanced Hybrid Sonar Framework: Novel simulation framework for imaging, profiling, sidescan, and singlebeam sonars, featuring simultaneous Ground Truth (GT) extraction and realistic stochastic noise modeling.
#. Sim-2-Real Focus: Realistic sensor noise and physics modeling designed to minimize the sim-to-real gap for seamless algorithm deployment.
#. Multi-Agent Missions: Easily configure and scale complex multi-robot missions.
#. OpenAI Gym-like Python Interface: Simple installation and intuitive API for testing robotics algorithms and training Reinforcement Learning agents.
#. High Performance & Flexibility: Configurable execution speeds. Run visually or in headless mode. Pay a computational penalty only for the features you need.
#. Cross-Platform: Full support for Linux and Windows.

Attribution and Relevent Publications
"""""""""""""""""""""""""""""""""""""
If you use BiguaSim in your research, please cite the following publication:

General BiguaSim use:
::
      
   @inproceedings{11338720,
      author={Mateus, Matheus G. and De Oliveira, Guilherme C. and Reichow, Luis Henrique K. and Kolling, Alisson H. and Pinheiro, Pedro M. and Drews-Jr, Paulo L. J.},
      booktitle={2025 IEEE International Conference on Advanced Robotics (ICAR)}, 
      title={BiguaSim: A Hybrid Multi-Domain Simulator for Robotics High-Fidelity Simulation and Synthetic Dataset Generation}, 
      year={2025},
      volume={},
      number={},
      pages={169-174},
      keywords={Data collection;Robot sensing systems;Hybrid power systems;Complexity theory;Vehicle dynamics;Engines;Testing;Synthetic data},
      doi={10.1109/ICAR65334.2025.11338720}
   }

.. toctree::
   :maxdepth: 2
   :caption: BiguaSim Documentation

   usage/installation
   usage/getting-started
   usage/usage
   server/server
   packages/packages
   agents/agents
   sensors/sensors
   ardubridge/ardubridge
   examples/examples
   develop/develop
   changelog/changelog

.. toctree::
   :maxdepth: 3
   :caption: API Documentation

   biguasim/index
   biguasim/agents
   biguasim/environments
   biguasim/spaces
   biguasim/commands
   biguasim/biguasimclient
   biguasim/packagemanager
   biguasim/sensors
   biguasim/lcm
   biguasim/dynamics
   biguasim/shmem
   biguasim/ardubridge
   biguasim/util
   biguasim/exceptions
   biguasim/weather
   biguasim/server-api
   biguasim/client-api


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
