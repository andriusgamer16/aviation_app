"""Merges real sensors with simulated fallbacks into avionics frames.

Source priority per instrument (first available wins):
  attitude   Windows inclinometer > Windows accel(+gyro) fusion
             > micro:bit accel > simulator
  heading    Windows compass > micro:bit compass > simulator
  g / slip   Windows accelerometer > micro:bit accelerometer > simulator
  position   Windows Location (GPS/Wi-Fi) > simulator

Every data block in a frame carries a `src` tag ("inclinometer", "fusion",
"compass", "microbit", "win", "sim", ...) so the UI can annunciate exactly
which instruments run on live hardware and which are simulated.
"""
from __future__ import annotations

import logging
import math
import os
import time

from .fusion import (
    ComplementaryFilter,
    MountCalibration,
    angles_to_gravity,
    gravity_to_angles,
)
from .mag_compass import MagCompass
from .microbit_sensors import MicroBitSensors
from .simulator import FlightSimulator
from .windows_sensors import WindowsSensors

log = logging.getLogger("avionics.manager")

# Airspeed from a DC-motor wind generator on micro:bit pin P1.
# Generated voltage rises roughly linearly with prop RPM / airspeed;
# calibrate the gain against a known speed (kt per volt).
ASI_ENABLED = os.environ.get("AVIONICS_ASI", "on").lower() not in ("off", "0")
ASI_GAIN_KT_PER_V = float(os.environ.get("AVIONICS_ASI_GAIN", "40.0"))
ASI_ZERO_COUNTS = int(os.environ.get("AVIONICS_ASI_ZERO", "8"))  # noise floor

_NO_SENSORS = {
    "accelerometer": False,
    "gyroscope": False,
    "compass": False,
    "inclinometer": False,
    "gps": False,
}


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class Avionics:
    def __init__(self, force_sim: bool = False) -> None:
        self.force_sim = force_sim
        self.sensors = None if force_sim else WindowsSensors()
        self.microbit = None if force_sim else MicroBitSensors()
        self.sim = FlightSimulator()
        self.cal = MountCalibration()
        self.mag = MagCompass()                  # micro:bit raw magnetometer
        self.fusion = ComplementaryFilter()      # Windows accel (+gyro)
        self.mb_fusion = ComplementaryFilter()   # micro:bit accel (no gyro)
        self._last_t: float | None = None
        self._last_grav = None                   # raw gravity vec for LEVEL
        self._turn_rate = 0.0
        self._last_hdg: float | None = None
        self._last_hdg_src: str | None = None
        self._vs = 0.0
        self._last_alt: float | None = None
        self._last_alt_ts: float | None = None
        self._ias = 0.0

    async def start(self) -> None:
        if self.sensors:
            await self.sensors.start_gps()

    async def stop(self) -> None:
        if self.sensors:
            self.sensors.stop()
        if self.microbit:
            self.microbit.stop()

    def zero(self) -> dict:
        """Capture the current gravity direction as straight-and-level."""
        if self._last_grav is None:
            return {"calibrated": False, "reason": "attitude is simulated"}
        ok = self.cal.capture(self._last_grav)
        self.fusion.reset()
        self.mb_fusion.reset()
        log.info("mount calibration %s", "captured" if ok else "rejected")
        return {"calibrated": ok}

    def debug(self) -> dict:
        """Full compass-pipeline snapshot for the /debug page."""
        now = time.monotonic()
        mb = self.microbit.read() if self.microbit else None
        out = {
            "t": round(now, 3),
            "microbit": None,
            "magCal": self.mag.debug(),
            "mount": {"active": self.cal.active, "matrix": self.cal.m},
            "windows": dict(self.sensors.available) if self.sensors else None,
            "airspeed": {
                "enabled": ASI_ENABLED,
                "gainKtPerV": ASI_GAIN_KT_PER_V,
                "zeroCounts": ASI_ZERO_COUNTS,
                "iasKt": round(self._ias, 1),
            },
        }
        if mb is None:
            return out
        accel = mb["accel"]
        a1 = mb.get("a1")
        entry = {
            "ageS": round(now - mb["ts"], 2),
            "accelG": [round(v, 3) for v in accel],
            "gMag": round(math.sqrt(sum(v * v for v in accel)), 3),
            "a1": a1,
            "a1Volts": None if a1 is None else round(a1 * 3.3 / 1023.0, 3),
            "mag": None,
        }
        mag = mb.get("mag")
        if mag is not None:
            corrected = self.mag.corrected(mag)
            m_virt = self.cal.apply(corrected)
            g_virt = self.cal.apply(accel)
            tilt = MountCalibration()
            tilt_ok = tilt.capture(g_virt)
            mh = tilt.apply(m_virt) if tilt_ok else m_virt
            horiz = math.hypot(mh[0], mh[1])
            entry["mag"] = {
                "rawNt": [round(v, 0) for v in mag],
                "correctedNt": [round(v, 0) for v in corrected],
                "virtNt": [round(v, 0) for v in m_virt],
                "horizNt": [round(v, 0) for v in mh],
                # |B| should stay ~constant (~50000 nT in Europe) as the
                # board rotates once the hard-iron offsets are right.
                "magnitudeNt": round(math.sqrt(sum(v * v for v in corrected)), 0),
                "horizMagnitudeNt": round(horiz, 0),
                # Magnetic dip: ~ +70 deg in the Baltics.
                "dipDeg": round(math.degrees(math.atan2(-mh[2], horiz)), 1)
                if horiz > 1.0 else None,
                # Computed with tilt compensation (what the PFD uses) and
                # naively flat, for comparison.
                "headingTiltComp": round(
                    math.degrees(math.atan2(-mh[0], mh[1])) % 360.0, 1)
                if tilt_ok else None,
                "headingFlat": round(
                    math.degrees(math.atan2(-m_virt[0], m_virt[1])) % 360.0, 1),
            }
        out["microbit"] = entry
        return out

    def frame(self) -> dict:
        now = time.monotonic()
        dt = min(0.5, now - self._last_t) if self._last_t is not None else 0.05
        self._last_t = now
        sim = self.sim.step(dt)

        s = self.sensors
        avail = dict(s.available) if s else dict(_NO_SENSORS)
        avail["microbit"] = self.microbit.connected if self.microbit else False

        accel = s.read_accel() if s else None
        gyro = s.read_gyro() if s else None
        incl = s.read_inclinometer() if s else None
        mb = self.microbit.read() if self.microbit else None

        # Physical LEVEL request from the micro:bit's B button (uses the
        # gravity vector captured on the previous frame; at 20 Hz that is
        # indistinguishable from "now").
        if self.microbit and self.microbit.consume_level_request():
            self.zero()

        # --- attitude ----------------------------------------------------
        if incl is not None:
            grav = angles_to_gravity(incl[0], incl[1])
            pitch, roll = gravity_to_angles(self.cal.apply(grav))
            att_src = "inclinometer"
            self._last_grav = grav
        elif accel is not None:
            self._last_grav = accel
            v = self.cal.apply(accel)
            g_rot = self.cal.apply(gyro) if gyro is not None else None
            pitch, roll = self.fusion.update(v, g_rot, dt)
            att_src = "fusion"
        elif mb is not None:
            self._last_grav = mb["accel"]
            v = self.cal.apply(mb["accel"])
            pitch, roll = self.mb_fusion.update(v, None, dt)
            att_src = "microbit"
        else:
            pitch, roll = sim["pitch"], sim["roll"]
            att_src = "sim"
            self._last_grav = None

        # --- heading -------------------------------------------------------
        # micro:bit magnetic heading: computed here from the raw field
        # (tilt-compensated, auto hard-iron calibration), with the board's
        # own calibrated heading as a fallback for legacy firmware.
        mb_hdg = None
        if mb is not None and mb.get("mag") is not None:
            self.mag.observe(mb["mag"])
            mb_hdg = self.mag.heading(mb["mag"], mb["accel"], self.cal)

        cmp_reading = s.read_compass() if s else None
        if cmp_reading is not None:
            hdg_mag, hdg_true = cmp_reading
            hdg_src = "compass"
        elif mb_hdg is not None:
            hdg_mag, hdg_true = mb_hdg, None
            hdg_src = "microbit"
        else:
            hdg_mag, hdg_true = sim["hdg"], sim["hdg"]
            hdg_src = "sim"

        # --- body accelerations / g-load / slip -----------------------------
        if accel is not None:
            body = self.cal.apply(accel)
            acc_src = "accelerometer"
        elif mb is not None:
            body = self.cal.apply(mb["accel"])
            acc_src = "microbit"
        else:
            body = sim["accel"]
            acc_src = "sim"
        ax, ay, az = body
        if acc_src == "sim":
            g_load, slip = sim["gLoad"], sim["slipG"]
        else:
            g_load = math.sqrt(ax * ax + ay * ay + az * az)
            slip = ax  # lateral, in the re-leveled (aircraft) frame

        gyro_block = None
        if gyro is not None:
            gyro_block = {"x": round(gyro[0], 2), "y": round(gyro[1], 2),
                          "z": round(gyro[2], 2), "src": "gyroscope"}
        elif att_src == "sim":
            g = sim["gyro"]
            gyro_block = {"x": round(g[0], 2), "y": round(g[1], 2),
                          "z": round(g[2], 2), "src": "sim"}

        # --- turn rate (heading derivative works for any mounting) ----------
        if hdg_src == "sim":
            self._turn_rate = sim["turnRateDps"]
            self._last_hdg = None
        else:
            if (self._last_hdg is not None and self._last_hdg_src == hdg_src
                    and dt > 0):
                rate = _wrap180(hdg_mag - self._last_hdg) / dt
                self._turn_rate += (rate - self._turn_rate) * min(1.0, dt * 2.5)
            self._last_hdg = hdg_mag
        self._last_hdg_src = hdg_src

        # --- GPS -------------------------------------------------------------
        gps_fix = s.read_gps() if s else None
        if gps_fix is not None:
            gps = {
                "lat": gps_fix["lat"], "lon": gps_fix["lon"],
                "altM": gps_fix["altM"], "gsMps": gps_fix["gsMps"],
                "trkDeg": gps_fix["trkDeg"], "accM": gps_fix["accM"],
                "fixSrc": gps_fix.get("fixSrc", "unknown"),
                "ageS": round(now - gps_fix["ts"], 1), "src": "win",
            }
        else:
            gps = {
                "lat": sim["lat"], "lon": sim["lon"], "altM": sim["altM"],
                "gsMps": sim["gsMps"], "trkDeg": sim["trkDeg"],
                "accM": sim["accM"], "fixSrc": "sim",
                "ageS": 0.0, "src": "sim",
            }

        # --- vertical speed --------------------------------------------------
        # Differentiate only across *new* fixes (fix timestamps), never
        # against the 20 Hz frame dt: between fixes the cached altitude is
        # constant and dividing by dt would produce a sawtooth.
        if gps["src"] == "sim":
            self._vs = sim["vsMps"]
            self._last_alt = self._last_alt_ts = None
        else:
            alt, ts = gps["altM"], gps_fix["ts"]
            if alt is None:
                self._vs = 0.0
                self._last_alt = self._last_alt_ts = None
            elif self._last_alt_ts is None:
                self._last_alt, self._last_alt_ts = alt, ts
            elif ts > self._last_alt_ts:
                rate = (alt - self._last_alt) / (ts - self._last_alt_ts)
                self._vs += (rate - self._vs) * 0.4
                self._last_alt, self._last_alt_ts = alt, ts

        # --- airspeed (DC-motor wind generator on micro:bit P1) --------------
        ias_kt = None
        a1 = mb.get("a1") if mb is not None else None
        if ASI_ENABLED and a1 is not None:
            counts = max(0, a1 - ASI_ZERO_COUNTS)
            raw_kt = counts * (3.3 / 1023.0) * ASI_GAIN_KT_PER_V
            self._ias += (raw_kt - self._ias) * min(1.0, dt * 2.5)
            ias_kt = round(self._ias, 1)
        else:
            self._ias = 0.0
        ias = {
            "kt": ias_kt,
            "a1": a1,
            "volts": None if a1 is None else round(a1 * 3.3 / 1023.0, 3),
            "src": "mb-p1" if ias_kt is not None else "off",
        }

        srcs = (att_src, hdg_src, acc_src, gps["src"])
        mode = "sim" if all(x == "sim" for x in srcs) else (
            "live" if all(x != "sim" for x in srcs) else "mixed")

        return {
            "t": round(now, 3),
            "att": {"pitch": round(pitch, 2), "roll": round(roll, 2),
                    "src": att_src},
            "hdg": {"mag": round(hdg_mag, 1),
                    "true": None if hdg_true is None else round(hdg_true, 1),
                    "src": hdg_src},
            "acc": {"x": round(ax, 3), "y": round(ay, 3), "z": round(az, 3),
                    "g": round(g_load, 2), "slip": round(slip, 3),
                    "src": acc_src},
            "gyro": gyro_block,
            "turnRateDps": round(self._turn_rate, 2),
            "vsMps": round(self._vs, 2),
            "ias": ias,
            "gps": {k: (round(v, 6) if isinstance(v, float) else v)
                    for k, v in gps.items()},
            "status": {"mode": mode, "sensors": avail,
                       "zeroed": self.cal.active,
                       "mbMagCal": round(self.mag.progress, 2)},
        }
