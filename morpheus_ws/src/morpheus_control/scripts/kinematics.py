#!/usr/bin/env python3
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ChassisParams:
    # Physical dimensions of the rover chassis (metres / rad)
    wheel_base: float = 1.072          # front-to-rear axle distance
    wheel_radius: float = 0.125
    wheel_steering_y_offset: float = 0.0  # lateral offset of steering pivot from wheel center
    wheel_separation: float = 0.615    # left-to-right track width
    drive_gain: float = 10.0           # scales normalised [-1,1] cmd to controller units
    # deadzone must be < Nav2's minimum cmd (~0.05), otherwise legitimate
    # navigation velocities get swallowed as if they were joystick noise.
    deadzone: float = 0.01

    @property
    def steering_track(self) -> float:
        # Effective steering width after accounting for pivot offset
        return self.wheel_separation - 2 * self.wheel_steering_y_offset


def compute_drive(
    vx: float, vy: float, wz: float, params: ChassisParams
) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute steering positions and wheel velocities from cmd_vel.

    Drive mode selection:
      wz != 0 and vx != 0  →  Ackermann (arc turn while driving forward)
      wz != 0 and vx == 0  →  Pivot     (spin in place, minimal scrub)
      otherwise            →  Crab      (strafe / straight drive)

    Returns (pos[4], vel[4]) — front-left, front-right, rear-left, rear-right.
    """
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
    """
    Ackermann steering: compute per-wheel steer angle and speed for an arc turn.

    The instantaneous turn radius r is derived from the linear/angular velocity
    ratio. Multiplying by 2π converts from rad/s ratio to the arc-length radius
    used by the geometry (controller convention).

    Left/right wheels travel different radii (r ± track/2), so their steer angles
    and speeds differ — this is the defining property of Ackermann geometry that
    prevents tyre scrub on a rigid axle.
    """
    factor = p.drive_gain

    if abs(wz) >= 1e-5:
        r = abs(vx) / wz * 2 * math.pi   # turn radius in controller units
        r_bl = r + p.steering_track / 2.0  # left wheel radius (larger on left turn)
        r_br = r - p.steering_track / 2.0  # right wheel radius

        a_fl = math.atan(p.wheel_base / r_bl)  # front-left steer angle (rad)
        a_fr = math.atan(p.wheel_base / r_br)  # front-right steer angle (rad)

        # Correct for sign flip when a wheel's radius crosses zero (tight turn)
        if r_bl > 0 and r < 0:
            a_fl -= math.pi
        if r_br < 0 and r > 0:
            a_fr += math.pi

        # Map to controller position units (controller expects 0..π/2 range → scale by π/2≈1.57)
        pos[0] = a_fl * 1.57
        pos[1] = a_fr * 1.57

    # Velocity: each wheel's ground speed is proportional to its turn radius.
    # vel_steering_offset corrects for the pivot being laterally offset from the wheel.
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
    # Rear wheels share the same speed as the corresponding front wheel
    vel[2] = vel[0]
    vel[3] = vel[1]
    vel *= factor


def _pivot(
    wz: float, p: ChassisParams,
    pos: np.ndarray, vel: np.ndarray,
) -> None:
    """
    In-place rotation: all wheels steer to point tangent to the turn circle.

    The optimal steer angle is atan(wheel_base / steering_track), which makes
    each wheel perpendicular to the radius line — minimising tyre scrub.
    Front and rear wheels steer in opposite directions (X pattern).
    Left/right wheels spin in opposite directions to produce the net yaw.
    """
    factor = p.drive_gain
    ang = math.atan(p.wheel_base / p.steering_track)  # ~34° for default params
    # X-pattern steering: FL/RR negative, FR/RL positive
    pos[0] = -ang   # front-left
    pos[1] = ang    # front-right
    pos[2] = ang    # rear-left
    pos[3] = -ang   # rear-right
    # Left wheels spin backward, right wheels forward (for CCW rotation when wz>0)
    vel[0] = -wz
    vel[1] = wz
    vel[2] = -wz
    vel[3] = wz
    vel *= factor


def _crab(
    vx: float, vy: float, p: ChassisParams,
    pos: np.ndarray, vel: np.ndarray,
) -> None:
    """
    Crab (holonomic strafe): all wheels point in the same direction.

    The steer angle is the direction of the velocity vector. Angles beyond ±90°
    are wrapped by reversing the wheel spin instead — avoids mechanically
    impossible >90° steer positions and unnecessary 180° turns.
    """
    factor = p.drive_gain

    if vx > 0.0 or vy != 0.0:
        angle = math.atan2(vy, vx) if (vx != 0.0 or vy != 0.0) else 0.0
        # Wrap: if angle > 90°, flip to the equivalent <90° angle and reverse vel below
        if abs(angle) >= math.pi / 2:
            angle = -np.sign(angle) * (math.pi - abs(angle))
        pos[:] = angle  # all four wheels steer identically

    magnitude = math.hypot(vx, vy)
    sign = 1.0 if vx > 0.0 else -1.0 if vx < 0.0 else 1.0
    vel[:] = magnitude * sign * factor
