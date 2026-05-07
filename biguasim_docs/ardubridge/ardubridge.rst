ArduBridge Architecture
=======================

The **ArduBridge** module provides a robust, real-time, two-way communication link between the BiguaSim environment and ArduPilot's Software In The Loop (SITL) ecosystem. 

Unlike traditional plugins that are tightly coupled to the game engine, ArduBridge operates as a lightweight Python middleware. It is orchestrated by the ``ArduBiguaSimRunner``, which synchronizes the Unreal Engine 5 simulation ticks with the ArduPilot control loop via UDP.

High-Level Data Flow
--------------------

The bridge handles the simulation loop in four continuous steps over UDP (default port ``9002``):

1. **Receiving Actuator Commands:** The bridge listens for 40-byte binary servo packets from ArduPilot (identified by the magic number ``18458``). These packets contain raw PWM values (typically ranging from 1000 to 2000) for up to 16 channels.
2. **Translating PWM to Physics:** Raw PWM values are passed through the ``VehicleProfile``. Depending on the vehicle, these values are mapped to specific simulation actuators, mathematically converted to physical units (e.g., RPM, angle degrees, or normalized thrust), and adjusted for correct motor spin direction.
3. **Engine Tick:** The ``ArduBiguaSimRunner`` passes these converted commands to the BiguaSim environment via ``env.step()``. Unreal Engine calculates the physics step and returns the updated sensor suite (Location, Velocity, IMU, Depth, etc.).
4. **Publishing State:** The bridge packs the new sensor data into a JSON payload formatted strictly to ArduPilot's SITL backend standards and sends it back to the Flight Controller Unit (FCU).

Coordinate Frame Conversions
----------------------------

One of the most critical roles of the ``frame.py`` module is translating between BiguaSim's native coordinate systems and ArduPilot's aeronautical standards in real-time.

* **BiguaSim Standards:** Uses **NWU** (North-West-Up) for the world and **GLU** (Forward-Left-Up) for the vehicle body.
* **ArduPilot Standards:** Expects **NED** (North-East-Down) for the world and **FRD** (Forward-Right-Down) for the vehicle body.

To bridge these domains, the system performs continuous 180° rotations about the x-axis (Roll).
For attitude dynamics (Quaternions), the canonical conversion used is:
``R_frd2ned = Rx(π) * R_glu2nwu * Rx(π)``

Additionally, the bridge automatically handles:

* Converting Cartesian NWU meters to GPS Latitude/Longitude coordinates based on a configurable origin point.
* Converting upward Z-axis depth into absolute pressure (Pascals), assuming standard seawater density (1026 kg/m^3).

Vehicle Profiles & PWM Mapping
------------------------------

Because BiguaSim supports multi-domain operations (from Quadcopters to heavy ROVs and Torpedo AUVs), the bridge is vehicle-agnostic. 

Actuator logic is abstracted into the ``VEHICLE_REGISTRY`` inside ``vehicle.py``. Each vehicle profile defines:

* **Motor Mapping:** Which ArduPilot channel corresponds to which BiguaSim actuator index.
* **Converters:** Functional wrappers that translate bipolar (1100-1900) or unipolar (1000-2000) PWM signals into specific physical commands (e.g., converting a PWM signal into a precise 90 fin angle for the TorpedoAUV).
* **Warmup Guard:** A configurable frame delay (e.g., 1700 frames) that forces motors to zero during the initial SITL boot sequence to prevent erratic physics impulses.

Mocking & Testing
-----------------

For development and debugging without the overhead of rendering the UE5 environment, the module includes mocking scripts:

* ``mock_bridge_json.py``: Emulates the ArduPilot JSON SITL backend, generating fake IMU/GPS data and running a toy physics model to verify network stability.
* ``mock_bridge_mavlink.py``: Acts as a lightweight proxy, utilizing ``pymavlink`` to test heartbeat connections and basic SERVO_OUTPUT_RAW parsing.