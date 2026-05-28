"""
Coordinate frame conversions: BiguaSim (NWU world, GLU body) → ArduPilot (NED world, FRD body).

BiguaSim world: NWU  — x=North, y=West,    z=Up
BiguaSim body:  GLU  — x=forward, y=left,  z=up
ArduPilot world: NED — x=North, y=East,    z=Down
ArduPilot body:  FRD — x=forward, y=right, z=down

Both world and body conversions are a 180° rotation about the x-axis: Rx(π).
Quaternion chain: R_frd2ned = Rx(π) * R_glu2nwu * Rx(π)
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation as R

_EARTH_RADIUS_M = 6_371_000.0
_Rx180 = R.from_euler('x', np.pi)


def pos_nwu_to_ap(pos, gps_origin: tuple) -> list:
    """
    BiguaSim LocationSensor [x,y,z] (NWU, metres, z=up) → [lat_deg, lon_deg, alt_m_AMSL].
    alt_m is positive upward: submerged → negative, above water → positive.
    Feed as separate "latitude"/"longitude"/"altitude" scalars to the JSON SITL backend.
    """
    lat0, lon0 = gps_origin
    north, west, z_up = float(pos[0]), float(pos[1]), float(pos[2])
    east = -west                # NWU y=West → NED y=East
    alt  = z_up                 # NWU z=up already matches ArduPilot's positive-up altitude

    lat = lat0 + math.degrees(north / _EARTH_RADIUS_M)
    lon = lon0 + math.degrees(east / (_EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return [lat, lon, alt]


def vel_nwu_to_ned(vel) -> list:
    """BiguaSim VelocitySensor [vn, vw, vu] (NWU, z=up) → NED [vn, ve, vd] (z=down)."""
    vn, vw, vu = float(vel[0]), float(vel[1]), float(vel[2])
    return [vn, -vw, -vu]


def imu_glu_to_frd(imu_data) -> tuple:
    """
    BiguaSim IMUSensor [[ax,ay,az],[p,q,r]] (GLU body) → ([ax,ay,az],[p,q,r]) (FRD body).
    GLU → FRD: negate y and z axes.
    """
    ax, ay, az = float(imu_data[0][0]), float(imu_data[0][1]), float(imu_data[0][2])
    p,  q,  r  = float(imu_data[1][0]), float(imu_data[1][1]), float(imu_data[1][2])
    return [ax, -ay, -az], [p, -q, -r]


def quat_glu2nwu_to_frd2ned(q_xyzw) -> list:
    """
    BiguaSim DynamicsSensor quaternion [x,y,z,w] (GLU→NWU) → [w,x,y,z] (FRD→NED).
    R_frd2ned = Rx(π) * R_glu2nwu * Rx(π)
    scipy as_quat() returns [x,y,z,w]; ArduPilot JSON wants [w,x,y,z].
    Canonical form enforced: w >= 0 (q and -q represent the same rotation).
    """
    R_glu2nwu = R.from_quat(q_xyzw)  # scipy: [x,y,z,w]
    R_frd2ned = _Rx180 * R_glu2nwu * _Rx180
    x, y, z, w = R_frd2ned.as_quat()
    if w < 0.0:  # enforce canonical form
        w, x, y, z = -w, -x, -y, -z
    return [float(w), float(x), float(y), float(z)]


def depth_to_pressure(depth_z_up: float, rho: float = 1026.0) -> float:
    """
    BiguaSim DepthSensor z_up (metres, positive=up) → absolute pressure (Pa).
    Assumes seawater density rho=1026 kg/m³.
    """
    depth_m = max(0.0, -depth_z_up)  # z_up negative when submerged
    return 101325.0 + rho * 9.80665 * depth_m


def meters_to_gps(north_m: float, east_m: float,
                  lat0_deg: float, lon0_deg: float) -> tuple:
    """Local NED offset (metres) → GPS lat/lon (degrees)."""
    lat = lat0_deg + math.degrees(north_m / _EARTH_RADIUS_M)
    lon = lon0_deg + math.degrees(east_m / (_EARTH_RADIUS_M * math.cos(math.radians(lat0_deg))))
    return lat, lon
