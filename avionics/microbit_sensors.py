"""BBC micro:bit as an external sensor pod (accelerometer + compass).

The micro:bit runs firmware/microbit_main.py (flash it with
`python -m uflash firmware/microbit_main.py`), which streams lines over
USB serial at 115200 baud, ~25 Hz:

    MB,<ax>,<ay>,<az>,<mx>,<my>,<mz>,<heading>,<btnB>

ax/ay/az in milli-g; mx/my/mz raw magnetometer field in nano-tesla
(consumed by mag_compass.py — no on-device calibration needed); heading
0-359 once the on-device compass is calibrated (else -1), kept as a
fallback; btnB=1 on a B-button press (used as a physical "LEVEL"
request). The older 6-field format without the magnetometer is still
accepted.

A daemon thread auto-detects the board by USB VID:PID (0D28:0204,
DAPLink CDC), reconnects on unplug, and hands the freshest sample over
under a lock. Data older than MAX_AGE_S counts as absent, so a paused
stream (e.g. during the on-device compass-calibration game started with
button A) degrades cleanly to the next source.
"""
from __future__ import annotations

import logging
import os
import threading
import time

log = logging.getLogger("avionics.microbit")

try:
    import serial
    from serial.tools import list_ports
    _HAVE_SERIAL = True
except ImportError:
    _HAVE_SERIAL = False

VID, PID = 0x0D28, 0x0204  # ARM DAPLink CDC (micro:bit v1 and v2)
BAUD = 115200
MAX_AGE_S = 1.0

# micro:bit accel axes -> the Windows device convention used by fusion.py
# (X right, Y toward top edge, Z out of the display; flat face-up = -1 g Z).
# micro:bit X (positive tilting right) and Z (-1024 mg face-up) already
# match; micro:bit Y is documented "positive tilting towards you", i.e.
# toward the pin edge, so it flips. If pitch moves the wrong way on your
# unit, set AVIONICS_MB_Y_SIGN=1.
_Y_SIGN = int(os.environ.get("AVIONICS_MB_Y_SIGN", "-1"))


class MicroBitSensors:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._data: dict | None = None  # {"accel": Vec3 g, "heading": float|None, "ts": float}
        self._level_req = False
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        if _HAVE_SERIAL:
            self._thread = threading.Thread(
                target=self._run, name="microbit-serial", daemon=True
            )
            self._thread.start()
        else:
            log.info("pyserial not installed; micro:bit support disabled")

    # ------------------------------------------------------------------
    def _find_port(self) -> str | None:
        try:
            for p in list_ports.comports():
                if p.vid == VID and p.pid == PID:
                    return p.device
        except Exception as exc:
            log.debug("port scan failed: %s", exc)
        return None

    def _run(self) -> None:
        announced_waiting = False
        while not self._stop.is_set():
            port = self._find_port()
            if port is None:
                if not announced_waiting:
                    log.info("no micro:bit found; watching for one (hot-plug)")
                    announced_waiting = True
                self._stop.wait(3.0)
                continue
            announced_waiting = False
            try:
                with serial.Serial(port, BAUD, timeout=1.0) as ser:
                    log.info("micro:bit connected on %s", port)
                    ser.reset_input_buffer()
                    while not self._stop.is_set():
                        raw = ser.readline()
                        if not raw:
                            continue  # timeout; keep the port open
                        self._parse(raw.decode("ascii", "ignore").strip())
            except (OSError, serial.SerialException) as exc:
                log.warning("micro:bit disconnected: %s", exc)
                with self._lock:
                    self._data = None
                self._stop.wait(2.0)

    def _parse(self, line: str) -> None:
        if not line.startswith("MB,"):
            return
        parts = line.split(",")
        try:
            if len(parts) >= 9:  # current format, with raw magnetometer
                ax, ay, az = int(parts[1]), int(parts[2]), int(parts[3])
                mx, my, mz = int(parts[4]), int(parts[5]), int(parts[6])
                mag = (float(mx), _Y_SIGN * float(my), float(mz))
                hdg = int(parts[7])
                btn_b = parts[8] == "1"
            elif len(parts) >= 6:  # legacy format, accel + heading only
                ax, ay, az = int(parts[1]), int(parts[2]), int(parts[3])
                mag = None
                hdg = int(parts[4])
                btn_b = parts[5] == "1"
            else:
                return
        except ValueError:
            return
        sample = {
            "accel": (ax / 1024.0, _Y_SIGN * ay / 1024.0, az / 1024.0),
            "mag": mag,
            "heading": float(hdg) if 0 <= hdg < 360 else None,
            "ts": time.monotonic(),
        }
        with self._lock:
            self._data = sample
            if btn_b:
                self._level_req = True

    # ------------------------------------------------------------------
    def read(self) -> dict | None:
        """Freshest sample, or None when absent/stale."""
        with self._lock:
            d = self._data
        if d is None or time.monotonic() - d["ts"] > MAX_AGE_S:
            return None
        return d

    def consume_level_request(self) -> bool:
        """True once per B-button press on the board."""
        with self._lock:
            req, self._level_req = self._level_req, False
        return req

    @property
    def connected(self) -> bool:
        return self.read() is not None

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
