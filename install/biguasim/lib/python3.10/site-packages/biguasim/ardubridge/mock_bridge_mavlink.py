"""
This script emulates the connection between a Python script and ArduPilot STIL.
We use MAVLink to make this connection, listening for actuator data from ArduPilot
on UDP 5502. Simulates GPS and IMU and sensor data. Send it back to ArduPilot via
UDP 5501.
"""

import random
import time

from pymavlink import mavutil

# This connects to the port ArduPilot is listening to for sensor input
mav = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
mav.wait_heartbeat()
print(f"Connected to system {mav.target_system}")

position = [0.0, 0.0, 30.0]  # lat, lon, alt (in meters)
velocity = [0.0, 0.0, 0.0]

while True:
    # Read PWM output from ArduPilot
    msg = mav.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
    if msg:
        pwm = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
        print("PWM:", pwm)

        # Fake dynamics (very basic)
        throttle = sum(pwm) / (4 * 1000.0)
        velocity[2] = (throttle - 1.5) * 2.0  # Simulate vertical motion
        position[2] += velocity[2] * 0.05  # Integrate altitude

    # --- GPS_RAW_INT ---
    mav.mav.gps_raw_int_send(
        int(time.time() * 1e6),  # time_usec
        3,  # fix_type (3D)
        int(0.000 * 1e7),  # lat
        int(-122.4194 * 1e7),  # lon
        int(0 * 1000),  # alt (in mm)
        0,
        0,
        0,
        0,  # eph, epv, vel, cog
        10,  # satellites_visible
        0,  # alt_ellipsoid
    )

    # --- RAW_IMU ---
    mav.mav.raw_imu_send(
        int(time.time() * 1e6),  # time_usec
        0,
        0,
        int(-velocity[2] * 1000),  # acc x/y/z
        0,
        0,
        0,  # gyro x/y/z
        0,
        0,
        0,  # mag x/y/z (not used)
    )

    time.sleep(0.05)
