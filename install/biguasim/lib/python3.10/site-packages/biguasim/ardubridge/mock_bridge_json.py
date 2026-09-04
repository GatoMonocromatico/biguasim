import json
import socket
import struct
import time

from pymavlink import mavutil

# --- Setup UDP socket for JSON SITL backend ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", 9002))  # This port must match ArduPilot's JSON backend
sock.settimeout(0.1)
last_frame = -1
connected = False

import random


def get_fake_accel():
    return [
        random.gauss(0, 0.1),  # x
        random.gauss(0, 0.1),  # y
        9.81 + random.gauss(0, 0.1),  # z
    ]


# Initial fake state
timestamp = 0.0
position = [0, 0, 0]
velocity = [0, 0, 0]
gyro = [0, 0, 0]
accel = get_fake_accel()
attitude = [0, 0, 0]
sensor_data = {
    "timestamp": timestamp,
    "imu": {"gyro": gyro, "accel_body": accel},
    "position": position,
    "velocity": velocity,
    "attitude": attitude,
}
print("JSON SITL backend listener ready on port 9002")


def send_sensor_data(sensor_data, addr):
    global timestamp
    timestamp += 0.05

    msg = "\n" + json.dumps(sensor_data, separators=(",", ":")) + "\n"
    sock.sendto(msg.encode("ascii"), addr)


# Now connect via MAVLink
print("🔌 Connecting MAVLink...")
mav = mavutil.mavlink_connection("udp:127.0.0.1:14550")
# mav.wait_heartbeat()
print(f"✅ Heartbeat from system {mav.target_system}, component {mav.target_component}")

# Change mode to GUIDED
mav.set_mode_apm("GUIDED")
time.sleep(5.5)

# Arm
mav.mav.command_long_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,
    0,
    0,
    0,
    0,
    0,
    0,
)
print("🚁 Arm command sent")

time.sleep(5.5)

while True:
    try:
        data, addr = sock.recvfrom(100)
    except socket.timeout:
        continue

    # --- Parse SITL binary output ---
    try:
        decoded = struct.unpack("HHI16H", data)
    except struct.error:
        print(f"Invalid packet size: {len(data)}")
        continue

    magic = decoded[0]
    frame_rate_hz = decoded[1]
    frame_count = decoded[2]
    pwm = decoded[3:]

    if magic != 18458:
        print(f"Unexpected magic: {magic}")
        continue

    if frame_count == last_frame:
        continue  # Duplicate frame
    elif frame_count < last_frame:
        print("Controller reset, resetting sim...")
        position = [0.0, 0.0, 0.0]
        velocity = [0.0, 0.0, 0.0]
    elif last_frame >= 0 and frame_count > last_frame + 1:
        print(f"⚠️ Missed {frame_count - last_frame - 1} frame(s)")

    last_frame = frame_count

    if not connected:
        print(f"Connected to SITL at {addr}")
        connected = True

    # --- Simple physics model ---
    avg_throttle = sum(pwm[:4]) / (4 * 1000.0)  # normalize to [1, 2]
    velocity[2] = (avg_throttle - 1.5) * 2.0  # toy model for vertical speed
    position[2] += velocity[2] * (1.0 / frame_rate_hz)  # integrate down position
    position[1] += velocity[2] * 0.4 * (1.0 / frame_rate_hz)
    # --- Build JSON message ---
    sensor_data = {
        "timestamp": time.time(),
        "imu": {"gyro": gyro, "accel_body": get_fake_accel()},
        "position": position,
        "attitude": attitude,
        "velocity": velocity,
    }

    # Optional debug print
    print(f"[PWM] {pwm[:4]}  Alt: {-position[2]:.2f} m")
    send_sensor_data(sensor_data, addr)
    time.sleep(1.0 / frame_rate_hz)
