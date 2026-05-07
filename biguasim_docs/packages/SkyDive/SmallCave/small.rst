.. _small-cave:

==========
Small-Cave
==========

.. image:: small_cave_far.png
   :align: center
   :alt: Small-Cave Exterior View

Overview
========
The **Small-Cave** environment is a high-complexity cave system designed for micro-agent navigation and confined space exploration. The "Small" designation refers to the tunnel diameter, which is specifically scaled for small robotic agents (ROVs) rather than humans or large vehicles.

This environment is uniquely challenging as it features three distinct zones:
 **Fully Submerged:** Deep tunnel sections completely filled with water.
 
 **Semi-Submerged:** Areas with air pockets and varying water levels, ideal for surface-to-underwater transitions.

 **Dry Zones:** Upper cave chambers located entirely above the water line.

.. toctree::
   :maxdepth: 1
   :caption: Available Scenarios
   :glob:

Environment Dimensions & Coordinates
====================================
The cave system is centered within a specific coordinate grid to assist in SLAM (Simultaneous Localization and Mapping) validation.

Top-Down Layout
---------------
.. image:: small_cave_top_grid.png
   :align: center
   :alt: Top-Down Coordinate Grid

The cave footprint spans from **-300 to 300 units** on the longitudinal axis and **-300 to 300 units** laterally. The intricate network of interconnected tunnels is centered at the origin (0,0), providing a complex maze for navigation testing.

Vertical Profile and Water Line
-------------------------------
.. image:: small_cave_depth.png
   :align: center
   :alt: Vertical Profile

This cross-section view illustrates the vertical complexity of the system:
 **Water Surface (Z=0):** Represented by the green horizontal line.

 **Tunnel Depth:** Tunnels reach down to **-20 meters** below the surface.

 **Cave Ceiling:** Dry chambers extend up to **+10 meters** above the water level.

Landmarks and Waypoints
-----------------------
.. image:: small_cave_landmarks.png
   :align: center
   :alt: Cave Landmarks

Key strategic points have been identified for mission planning:
 **Submerged Entrance (1):** The main underwater access point for ROV deployment.

 **Transition Chamber (2):** A semi-submerged zone for testing sensor transitions between air and water.

 **Deep Siphon (3):** A narrow, fully flooded passage requiring precise maneuvering.

 **Dry Terminal (4):** The furthest reach of the dry cave sections.