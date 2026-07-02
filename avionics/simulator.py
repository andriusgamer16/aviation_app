"""Autonomous flight simulator.

Substitutes for any sensor the machine doesn't have, so the PFD is always
alive. Flies gentle, coordinated maneuvers: it picks a new target bank /
vertical speed / airspeed every 8-20 s and eases toward it, integrating
heading and position like a point-mass aircraft. Data it emits is marked
"sim" by the manager so the UI can annunciate it.
"""
from __future__ import annotations

import math
import random

R_EARTH = 6_371_000.0  # m
G = 9.80665            # m/s^2


class FlightSimulator:
    def __init__(
        self,
        lat: float = 54.6341,   # over Vilnius, Lithuania
        lon: float = 25.2858,
        alt_m: float = 1200.0,
        seed: int | None = None,
    ) -> None:
        self.rng = random.Random(seed)
        self.lat, self.lon, self.alt = lat, lon, alt_m
        self.hdg = self.rng.uniform(0.0, 360.0)
        self.tas = 55.0        # m/s ~ 107 kt
        self.roll = 0.0
        self.pitch = 1.5
        self.vs = 0.0
        self.turn_rate = 0.0
        self.g_load = 1.0
        self._roll_tgt = 0.0
        self._vs_tgt = 0.0
        self._spd_tgt = self.tas
        self._next_maneuver = 5.0
        self.t = 0.0

    def step(self, dt: float) -> dict:
        self.t += dt
        if self.t >= self._next_maneuver:
            self._next_maneuver = self.t + self.rng.uniform(8.0, 20.0)
            self._roll_tgt = self.rng.uniform(-25.0, 25.0)
            self._vs_tgt = self.rng.uniform(-3.0, 3.0)      # m/s
            self._spd_tgt = self.rng.uniform(46.0, 67.0)    # 90-130 kt

        def approach(value: float, target: float, rate: float) -> float:
            step = rate * dt
            delta = target - value
            return target if abs(delta) <= step else value + math.copysign(step, delta)

        self.roll = approach(self.roll, self._roll_tgt, 8.0)
        self.vs = approach(self.vs, self._vs_tgt, 1.0)
        self.tas = approach(self.tas, self._spd_tgt, 0.8)

        # Coordinated turn: rate = g * tan(bank) / TAS
        self.turn_rate = math.degrees(G * math.tan(math.radians(self.roll)) / self.tas)
        self.hdg = (self.hdg + self.turn_rate * dt) % 360.0
        climb = math.degrees(math.asin(max(-1.0, min(1.0, self.vs / self.tas))))
        self.pitch = climb + 1.5  # cruise deck angle
        self.alt = max(0.0, self.alt + self.vs * dt)

        dist = self.tas * dt
        brg = math.radians(self.hdg)
        self.lat += math.degrees(dist * math.cos(brg) / R_EARTH)
        self.lon += math.degrees(
            dist * math.sin(brg) / (R_EARTH * math.cos(math.radians(self.lat)))
        )

        self.g_load = 1.0 / max(0.2, math.cos(math.radians(self.roll)))

        # Light turbulence so the display breathes.
        turb = (
            0.50 * math.sin(self.t * 1.31)
            + 0.30 * math.sin(self.t * 2.71 + 1.7)
            + 0.20 * math.sin(self.t * 4.63 + 0.4)
        )
        slip = 0.03 * math.sin(self.t * 0.9 + 2.1)

        return {
            "pitch": self.pitch + 0.35 * turb,
            "roll": self.roll + 0.80 * turb,
            "hdg": self.hdg,
            "turnRateDps": self.turn_rate,
            "gLoad": self.g_load + 0.02 * turb,
            "slipG": slip,
            # Synthetic device-frame values, Windows sensor convention
            # (right-hand rule: a right turn is negative rate about +Z).
            "accel": (slip, 0.01 * turb, -self.g_load),
            "gyro": (0.0, 0.0, -self.turn_rate),
            "lat": self.lat,
            "lon": self.lon,
            "altM": self.alt,
            "gsMps": self.tas,
            "trkDeg": self.hdg,
            "vsMps": self.vs,
            "accM": 5.0,
        }
