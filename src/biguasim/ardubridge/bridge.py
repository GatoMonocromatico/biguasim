"""
ArduPilot JSON SITL UDP bridge — vehicle-agnostic protocol handler.

Receives 40-byte servo packets from ArduPilot, converts PWM to simulator
commands, converts simulator state to ArduPilot JSON, and sends it back.
No BiguaSim dependency; wired to the environment by ArduBiguaSimRunner.
"""

from __future__ import annotations

import json
import socket
import struct
from typing import Optional

import numpy as np

from .frame import (
    depth_to_pressure,
    imu_glu_to_frd,
    pos_nwu_to_ap,
    quat_glu2nwu_to_frd2ned,
    vel_nwu_to_ned,
)
from .vehicle import VehicleProfile

_SERVO_PACKET_FMT = "<HHI16H"
_SERVO_PACKET_SIZE = struct.calcsize(_SERVO_PACKET_FMT)  # 40 bytes
_SERVO_MAGIC = 18458


class ArduPilotBridge:
    """
    UDP bridge implementing the ArduPilot JSON SITL FDM protocol.
    
    Responsible for establishing two-way communication between ArduPilot and the simulator,
    receiving actuator commands (PWM) and sending the vehicle's sensory data (state) back.
    """

    def __init__(
        self,
        profile: VehicleProfile,
        address: str = "127.0.0.1",
        port: int = 9002,
        gps_origin: tuple = (33.810313, -118.393867),
    ):
        
        """
        Initializes the UDP communication bridge.

        :param profile: Vehicle profile containing motor mapping, converters, and constraints.
        :type profile: VehicleProfile
        :param address: Local IP address to bind the UDP socket. Defaults to "127.0.0.1".
        :type address: str
        :param port: Local port to listen for ArduPilot packets. Defaults to 9002.
        :type port: int
        :param gps_origin: Tuple containing the (Latitude, Longitude) of the simulator's world origin.
        :type gps_origin: tuple
        """

        self._profile = profile
        self._address = address
        self._port = port
        self._gps_origin = gps_origin

        self._fcu_address: Optional[str] = None
        self._fcu_port: Optional[int] = None
        self._online = False
        self._timeout_count = 0
        self._last_frame = -1

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._recv_buf = bytearray(_SERVO_PACKET_SIZE)  # pre-allocated; avoid hot-loop GC

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def bind(self) -> None:
        """
        Binds the UDP socket and drains any stale packets. 
        
        .. warning::
           Call this method only once before starting the execution loop (`run`).

        :raises RuntimeError: If it fails to bind to the specified UDP socket.
        """
        try:
            self._sock.bind((self._address, self._port))
            print(f"ArduPilot bridge bound @ {self._address}:{self._port}")
        except socket.error as exc:
            raise RuntimeError(f"Failed to bind UDP socket: {exc}") from exc
        self._drain_stale_packets()
        print("Drained stale packets – waiting for ArduPilot...")

    def receive_pwm(self) -> tuple:
        """
        Non-blocking receive of one servo packet (PWM).

        Dynamically adjusts the timeout depending on the connection status (`_online`).

        :returns: A tuple containing `(frame_count, pwm_array)`. Returns `(None, None)` on timeout or error.
        :rtype: tuple
        """
        self._sock.settimeout(0.001 if not self._online else 0.01)
        try:
            nbytes, (addr, port) = self._sock.recvfrom_into(self._recv_buf)
        except socket.timeout:
            self._handle_timeout()
            return None, None
        except OSError:
            return None, None

        if nbytes != _SERVO_PACKET_SIZE:
            return None, None

        if self._fcu_address is None:
            self._fcu_address, self._fcu_port = addr, port
            print(f"ArduPilot connected @ {addr}:{port}")

        unpacked = struct.unpack_from(_SERVO_PACKET_FMT, self._recv_buf)
        magic, _frame_rate, frame_count = unpacked[0], unpacked[1], unpacked[2]
        pwm = np.array(unpacked[3:], dtype=np.float32)

        if magic != _SERVO_MAGIC:
            print(f"Unexpected magic: {magic}")
            return None, None

        self._check_frame_continuity(frame_count)
        if not self._online:
            print("ArduPilot online")
            self._online = True
        self._timeout_count = 0
        self._last_frame = frame_count
        return frame_count, pwm

    def pwm_to_motor_cmds(self, pwm: np.ndarray, frame_count: int) -> list:
        """
        Converts raw PWM values received from ArduPilot into scaled motor commands for the simulator.

        Applies the warmup guard (`warmup_frames`), motor mapping, mathematical PWM converters, 
        and directional signs configured in the `VehicleProfile`.

        :param pwm: NumPy array containing the 16 PWM channels received from ArduPilot.
        :type pwm: np.ndarray
        :param frame_count: The current frame counter used to check the warmup phase.
        :type frame_count: int
        :returns: A list of floats containing the final normalized commands for each motor.
        :rtype: list
        """
        if frame_count < self._profile.warmup_frames:
            return [0.0] * self._profile.num_motors

        return [
            self._profile.motor_signs[i]
            * self._profile.pwm_converters[i](pwm[self._profile.motor_mapping[i]])
            for i in range(self._profile.num_motors)
        ]

    def build_json_state(self, agent_state: dict, sim_time: float) -> dict:
        """
        Converts the raw agent state in the simulator (NWU/GLU standard) to the ArduPilot 
        JSON state format (NED/FRD standard).

        :param agent_state: Dictionary containing BiguaSim sensor data (Location, Velocity, IMU, Depth, etc.).
        :type agent_state: dict
        :param sim_time: Current simulation time in seconds.
        :type sim_time: float
        :returns: Formatted dictionary ready to be serialized into JSON.
        :rtype: dict
        """
        accel, gyro = imu_glu_to_frd(agent_state["IMUSensor"])
        pos = pos_nwu_to_ap(agent_state["LocationSensor"].tolist(), self._gps_origin)
        vel = vel_nwu_to_ned(agent_state["VelocitySensor"].tolist())
        quat = quat_glu2nwu_to_frd2ned(agent_state["DynamicsSensor"][-4:].tolist())

        state = {
            "timestamp": sim_time,
            "imu": {"gyro": gyro, "accel_body": accel},
            "latitude": pos[0],
            "longitude": pos[1],
            "altitude": pos[2],
            "velocity": vel,
            "quaternion": quat,
        }

        # print(f"\
        #     'latitude': {pos[0]},\
        #     'longitude': {pos[1]},\
        #     'altitude': {pos[2]},\
        #     'quaternion:' {quat}"
            
        # );

        # if "DepthSensor" in agent_state:
        #     depth_val = agent_state["DepthSensor"]
        #     z_up = float(depth_val[0]) if hasattr(depth_val, "__len__") else float(depth_val)
        #     state["pressure"] = depth_to_pressure(z_up)

        return state

    def send_state(self, json_state: dict) -> None:
        """
        Serializes the state dictionary into JSON and sends it via UDP to the Flight Controller Unit (FCU).

        :param json_state: Dictionary generated by the :meth:`build_json_state` method.
        :type json_state: dict
        """
        if self._fcu_address is None:
            return
        payload = ("\n" + json.dumps(json_state, separators=(",", ":")) + "\n").encode("utf-8")
        self._sock.sendto(payload, (self._fcu_address, self._fcu_port))

    @property
    def is_online(self) -> bool:
        """
        Checks the current connection status with ArduPilot.

        :returns: ``True`` if ArduPilot is actively sending packets, ``False`` otherwise.
        :rtype: bool
        """
        return self._online

    def close(self) -> None:
        """
        Closes the UDP communication socket.
        """
        self._sock.close()

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _drain_stale_packets(self) -> None:
        """
        Internal method to empty the UDP receive buffer and avoid processing old packets.
        """
        self._sock.setblocking(False)
        try:
            while True:
                self._sock.recvfrom(_SERVO_PACKET_SIZE)
        except BlockingIOError:
            pass
        self._sock.setblocking(True)

    def _handle_timeout(self) -> None:
        """
        Manages communication failures and handles timeout disconnections.
        """
        if self._online:
            self._timeout_count += 1
            if self._timeout_count > 5:
                print("ArduPilot connection timeout — resetting.")
                self._online = False
                self._timeout_count = 0

    def _check_frame_continuity(self, frame_count: int) -> None:
        """
        Checks the integrity of the received packet sequence (detects frame drops or FCU resets).

        :param frame_count: The frame counter received in the current packet.
        :type frame_count: int
        """
        if frame_count < self._last_frame:
            print("ArduPilot reset detected")
        elif self._last_frame >= 0 and frame_count > self._last_frame + 1:
            print(f"Missed {frame_count - self._last_frame - 1} frame(s)")
