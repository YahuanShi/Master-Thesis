#!/usr/bin/env python3
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ChassisParams:
    wheel_base: float = 1.072
    wheel_radius: float = 0.125
    wheel_steering_y_offset: float = 0.0
    wheel_separation: float = 0.615
    drive_gain: float = 10.0
    deadzone: float = 0.05

    @property
    def steering_track(self) -> float:
        return self.wheel_separation - 2 * self.wheel_steering_y_offset


def compute_drive(
    vx: float, vy: float, wz: float, params: ChassisParams
) -> tuple[np.ndarray, np.ndarray]:
    """Compute steering positions and wheel velocities from cmd_vel."""
    pos = np.zeros(4)
    vel = np.zeros(4)

    if abs(vx) < params.deadzone:
        vx = 0.0
    if abs(vy) < params.deadzone:
        vy = 0.0
    if abs(wz) < params.deadzone:
        wz = 0.0

    if wz != 0.0:
        if vx != 0.0:
            _ackermann(vx, wz, params, pos, vel)
        else:
            _pivot(wz, params, pos, vel)
    else:
        _crab(vx, vy, params, pos, vel)

    return pos, vel


def _ackermann(
    vx: float, wz: float, p: ChassisParams,
    pos: np.ndarray, vel: np.ndarray,
) -> None:
    factor = p.drive_gain

    if abs(wz) >= 1e-5:
        r = abs(vx) / wz * 2 * math.pi
        r_bl = r + p.steering_track / 2.0
        r_br = r - p.steering_track / 2.0

        a_fl = math.atan(p.wheel_base / r_bl)
        a_fr = math.atan(p.wheel_base / r_br)

        if r_bl > 0 and r < 0:
            a_fl -= math.pi
        if r_br < 0 and r > 0:
            a_fr += math.pi

        pos[0] = a_fl * 1.57
        pos[1] = a_fr * 1.57

    vel_steering_offset = wz * p.wheel_steering_y_offset
    sign = np.sign(vx) if vx != 0.0 else 1.0

    vel[0] = (
        sign * math.hypot(vx - wz * p.steering_track / 2.0,
                          wz * p.wheel_base / 2.0)
        - vel_steering_offset
    )
    vel[1] = (
        sign * math.hypot(vx + wz * p.steering_track / 2.0,
                          wz * p.wheel_base / 2.0)
        + vel_steering_offset
    )
    vel[2] = vel[0]
    vel[3] = vel[1]
    vel *= factor


def _pivot(
    wz: float, p: ChassisParams,
    pos: np.ndarray, vel: np.ndarray,
) -> None:
    factor = p.drive_gain
    ang = math.atan(p.wheel_base / p.steering_track)
    pos[0] = -ang
    pos[1] = ang
    pos[2] = ang
    pos[3] = -ang
    vel[0] = -wz
    vel[1] = wz
    vel[2] = -wz
    vel[3] = wz
    vel *= factor


def _crab(
    vx: float, vy: float, p: ChassisParams,
    pos: np.ndarray, vel: np.ndarray,
) -> None:
    factor = p.drive_gain

    if vx > 0.0 or vy != 0.0:
        angle = math.atan2(vy, vx) if (vx != 0.0 or vy != 0.0) else 0.0
        if abs(angle) >= math.pi / 2:
            angle = -np.sign(angle) * (math.pi - abs(angle))
        pos[:] = angle

    magnitude = math.hypot(vx, vy)
    sign = 1.0 if vx > 0.0 else -1.0 if vx < 0.0 else 1.0
    vel[:] = magnitude * sign * factor
