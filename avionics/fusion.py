"""Attitude estimation and mounting calibration.

Axis convention (Windows.Devices.Sensors, device in natural orientation):
X to the right edge, Y to the top edge, Z out of the screen. A device lying
flat face-up at rest reads roughly (0, 0, -1) g. Aircraft convention:
nose-up pitch positive, right-bank roll positive.
"""
from __future__ import annotations

import math

Vec3 = tuple[float, float, float]

_TARGET: Vec3 = (0.0, 0.0, -1.0)  # gravity in the flat face-up rest frame


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def gravity_to_angles(v: Vec3) -> tuple[float, float]:
    """(pitch, roll) in degrees from a gravity vector in device frame."""
    ax, ay, az = v
    pitch = math.degrees(math.atan2(-ay, math.hypot(ax, az)))
    roll = math.degrees(math.atan2(ax, -az))
    return pitch, roll


def angles_to_gravity(pitch_deg: float, roll_deg: float) -> Vec3:
    """Unit gravity vector for the given attitude (inverse of the above)."""
    p = math.radians(pitch_deg)
    r = math.radians(roll_deg)
    return (
        math.cos(p) * math.sin(r),
        -math.sin(p),
        -math.cos(p) * math.cos(r),
    )


class MountCalibration:
    """Rotation that re-levels an arbitrarily mounted device.

    capture() records the current gravity direction; apply() then rotates
    subsequent sensor vectors (accelerometer AND gyroscope) so that
    direction maps to (0, 0, -1) — the flat face-up rest reading. This is
    valid for any static mounting (flat on a desk, propped upright like a
    panel), unlike per-axis angle offsets which invert pitch and saturate
    roll for near-vertical mounts. Yaw about gravity is unobservable from
    gravity alone and is left unchanged.
    """

    def __init__(self) -> None:
        self.m: list[Vec3] | None = None  # row-major 3x3

    @property
    def active(self) -> bool:
        return self.m is not None

    def clear(self) -> None:
        self.m = None

    def capture(self, g: Vec3) -> bool:
        n = math.sqrt(_dot(g, g))
        if n < 1e-6:
            return False
        u = (g[0] / n, g[1] / n, g[2] / n)
        c = _dot(u, _TARGET)
        if c > 1.0 - 1e-9:
            self.m = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
            return True
        if c < -1.0 + 1e-9:
            # Antiparallel (device face-down): 180 deg about X.
            self.m = [(1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)]
            return True
        axis = _cross(u, _TARGET)
        s = math.sqrt(_dot(axis, axis))  # = sin(theta), theta = angle u->target
        kx, ky, kz = axis[0] / s, axis[1] / s, axis[2] / s
        t = 1.0 - c
        # Rodrigues rotation matrix for angle theta about (kx, ky, kz).
        self.m = [
            (c + kx * kx * t, kx * ky * t - kz * s, kx * kz * t + ky * s),
            (ky * kx * t + kz * s, c + ky * ky * t, ky * kz * t - kx * s),
            (kz * kx * t - ky * s, kz * ky * t + kx * s, c + kz * kz * t),
        ]
        return True

    def apply(self, v: Vec3) -> Vec3:
        if self.m is None:
            return v
        m = self.m
        return (_dot(m[0], v), _dot(m[1], v), _dot(m[2], v))


class ComplementaryFilter:
    """Pitch/roll from accelerometer, stabilized by a gyroscope when present.

    With a gyro: trusts the integrated rates short-term and the gravity
    vector long-term (time constant `tau`). Without a gyro (e.g. the
    micro:bit pod): low-passes the gravity angles (time constant `tau_lp`)
    so the display doesn't jitter with hand tremor and vibration.
    """

    def __init__(self, tau: float = 0.8, tau_lp: float = 0.25) -> None:
        self.tau = tau
        self.tau_lp = tau_lp
        self.pitch: float | None = None
        self.roll: float | None = None

    def reset(self) -> None:
        self.pitch = None
        self.roll = None

    def update(
        self,
        accel: Vec3,
        gyro_dps: Vec3 | None,
        dt: float,
    ) -> tuple[float, float]:
        pitch_acc, roll_acc = gravity_to_angles(accel)

        if self.pitch is None or dt <= 0 or dt > 0.5:
            self.pitch, self.roll = pitch_acc, roll_acc
        elif gyro_dps is None:
            k = dt / (self.tau_lp + dt)
            self.pitch += (pitch_acc - self.pitch) * k
            self.roll += (roll_acc - self.roll) * k
        else:
            gx, gy, _ = gyro_dps
            alpha = self.tau / (self.tau + dt)
            # Right-hand rule about device axes: +X rate raises the nose,
            # +Y rate drops the right edge (both positive aircraft-wise).
            self.pitch = alpha * (self.pitch + gx * dt) + (1 - alpha) * pitch_acc
            self.roll = alpha * (self.roll + gy * dt) + (1 - alpha) * roll_acc
        return self.pitch, self.roll
