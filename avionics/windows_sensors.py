"""Windows sensor access via WinRT (pywinrt).

All imports are guarded so the app still runs — in simulator mode — on
machines without the winrt packages or without any sensor hardware.

Motion sensors (accelerometer, gyrometer, compass, inclinometer) are polled
synchronously from the sampler loop; readings are cheap to fetch. GPS updates
arrive through the Geolocator position-changed event on a WinRT worker
thread, so the latest fix is handed over under a lock.
"""
from __future__ import annotations

import asyncio
import logging
import math
import threading
import time

log = logging.getLogger("avionics.sensors")

try:
    from winrt.windows.devices.sensors import (
        Accelerometer,
        Compass,
        Gyrometer,
        Inclinometer,
    )
    _HAVE_SENSORS = True
except Exception:  # ImportError or WinRT activation failure
    _HAVE_SENSORS = False

try:
    from winrt.windows.devices.geolocation import (
        GeolocationAccessStatus,
        Geolocator,
        PositionAccuracy,
        PositionStatus,
    )
    _HAVE_GEO = True
except Exception:
    _HAVE_GEO = False

# A cached fix older than this is treated as no fix at all (position events
# stopped arriving: provider lost, permission revoked, service wedged).
MAX_FIX_AGE_S = 60.0


def _nan_to_none(v) -> float | None:
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


_POSITION_SOURCE_NAMES = {
    "SATELLITE": "sat",
    "WI_FI": "wifi",
    "CELLULAR": "cell",
    "IP_ADDRESS": "ip",
    "UNKNOWN": "unknown",
    "DEFAULT": "default",
    "OBFUSCATED": "obfuscated",
}


def _init_apartment() -> None:
    # Older pywinrt/winsdk builds require explicit apartment initialization;
    # current winrt-runtime does it lazily. No-op when unavailable.
    try:
        from winrt import init_apartment  # type: ignore[attr-defined]

        init_apartment()
    except Exception:
        pass


class WindowsSensors:
    """Thin wrapper around the WinRT sensor objects."""

    def __init__(self) -> None:
        _init_apartment()
        self.accelerometer = None
        self.gyrometer = None
        self.compass = None
        self.inclinometer = None
        self._geolocator = None
        self._gps_lock = threading.Lock()
        self._gps: dict | None = None
        self._pos_token = None
        self._status_token = None
        self._fix_task: asyncio.Task | None = None
        self.gps_ready = False

        if not _HAVE_SENSORS:
            log.info("winrt sensor bindings not installed; no motion sensors")
            return
        for name, cls in (
            ("accelerometer", Accelerometer),
            ("gyrometer", Gyrometer),
            ("compass", Compass),
            ("inclinometer", Inclinometer),
        ):
            try:
                dev = cls.get_default()
            except Exception as exc:
                log.warning("%s init failed: %s", name, exc)
                dev = None
            if dev is not None:
                # Required for polling: without a ReportInterval the driver
                # may never power the sensor and get_current_reading() can
                # return None (or stale idle-rate data) forever.
                try:
                    dev.report_interval = max(dev.minimum_report_interval, 50)
                except Exception as exc:
                    log.warning("%s: could not set report interval: %s", name, exc)
            setattr(self, name, dev)
            log.info("%s: %s", name, "present" if dev else "not found")

    # ------------------------------------------------------------------
    # Motion sensors (synchronous polls)
    # ------------------------------------------------------------------
    def read_accel(self) -> tuple[float, float, float] | None:
        """Acceleration in g, Windows convention (flat face-up ≈ (0, 0, -1))."""
        if not self.accelerometer:
            return None
        try:
            r = self.accelerometer.get_current_reading()
        except Exception:
            return None
        if r is None:
            return None
        v = (r.acceleration_x, r.acceleration_y, r.acceleration_z)
        return None if any(math.isnan(x) for x in v) else v

    def read_gyro(self) -> tuple[float, float, float] | None:
        """Angular velocity in degrees/second about device X, Y, Z."""
        if not self.gyrometer:
            return None
        try:
            r = self.gyrometer.get_current_reading()
        except Exception:
            return None
        if r is None:
            return None
        v = (r.angular_velocity_x, r.angular_velocity_y, r.angular_velocity_z)
        return None if any(math.isnan(x) for x in v) else v

    def read_compass(self) -> tuple[float, float | None] | None:
        """(magnetic heading, true heading or None) in degrees."""
        if not self.compass:
            return None
        try:
            r = self.compass.get_current_reading()
        except Exception:
            return None
        if r is None:
            return None
        mag = _nan_to_none(r.heading_magnetic_north)
        if mag is None:
            return None
        return (mag, _nan_to_none(r.heading_true_north))

    def read_inclinometer(self) -> tuple[float, float, float] | None:
        """(pitch, roll, yaw) in degrees, already fused by the sensor stack."""
        if not self.inclinometer:
            return None
        try:
            r = self.inclinometer.get_current_reading()
        except Exception:
            return None
        if r is None:
            return None
        v = (r.pitch_degrees, r.roll_degrees, r.yaw_degrees)
        return None if any(math.isnan(x) for x in v) else v

    # ------------------------------------------------------------------
    # GPS / Windows location service
    # ------------------------------------------------------------------
    async def start_gps(self) -> None:
        if not _HAVE_GEO:
            log.info("winrt geolocation bindings not installed; no GPS")
            return
        try:
            status = await asyncio.wait_for(
                asyncio.ensure_future(Geolocator.request_access_async()), 10.0
            )
        except Exception as exc:
            log.warning("location access request failed: %s", exc)
            return
        if status != GeolocationAccessStatus.ALLOWED:
            log.warning(
                "location access not granted (%s); enable it under Windows "
                "Settings > Privacy & security > Location",
                status,
            )
            return
        geo = Geolocator()
        try:
            geo.desired_accuracy = PositionAccuracy.HIGH
        except Exception:
            pass
        try:
            geo.report_interval = 1000
        except Exception:
            pass  # device may enforce a larger minimum interval
        self._pos_token = geo.add_position_changed(self._on_position)
        self._status_token = geo.add_status_changed(self._on_status)
        self._geolocator = geo
        self.gps_ready = True
        # A cold fix can take up to ~60 s; never block server startup on it.
        # position_changed delivers fixes anyway; this just speeds up the
        # first one.
        self._fix_task = asyncio.create_task(self._initial_fix(geo))

    async def _initial_fix(self, geo) -> None:
        try:
            pos = await geo.get_geoposition_async()
            self._store_position(pos)
        except Exception as exc:
            log.warning("initial position fix failed: %s", exc)

    def stop(self) -> None:
        if self._fix_task is not None:
            self._fix_task.cancel()
            self._fix_task = None
        geo = self._geolocator
        if geo is not None:
            self._geolocator = None
            self.gps_ready = False
            try:
                if self._pos_token is not None:
                    geo.remove_position_changed(self._pos_token)
                if self._status_token is not None:
                    geo.remove_status_changed(self._status_token)
            except Exception as exc:
                log.debug("geolocator unsubscribe failed: %s", exc)

    def _on_position(self, sender, args) -> None:
        try:
            self._store_position(args.position)
        except Exception as exc:
            log.debug("position event error: %s", exc)

    def _on_status(self, sender, args) -> None:
        try:
            status = args.status
            if status in (PositionStatus.READY, PositionStatus.INITIALIZING):
                log.info("location status: %s", status.name)
                return
            # Disabled / NoData / NotAvailable: drop the cached fix so the
            # app degrades to sim instead of presenting stale data as live.
            log.warning("location status: %s; clearing cached fix", status.name)
            with self._gps_lock:
                self._gps = None
        except Exception as exc:
            log.debug("status event error: %s", exc)

    def _store_position(self, pos) -> None:
        c = pos.coordinate
        p = c.point.position
        # WinRT reports "unknown" as either None (empty IReference) or NaN.
        # NaN must never reach json.dumps: browsers reject NaN in JSON.
        alt_acc = _nan_to_none(getattr(c, "altitude_accuracy", None))
        alt = _nan_to_none(p.altitude)
        if alt_acc is None:
            alt = None  # e.g. Wi-Fi/IP fixes report a meaningless 0.0
        try:
            fix_src = _POSITION_SOURCE_NAMES.get(c.position_source.name, "unknown")
        except Exception:
            fix_src = "unknown"
        fix = {
            "lat": _nan_to_none(p.latitude),
            "lon": _nan_to_none(p.longitude),
            "altM": alt,
            "accM": _nan_to_none(c.accuracy),
            "gsMps": _nan_to_none(c.speed),
            "trkDeg": _nan_to_none(c.heading),
            "fixSrc": fix_src,
            "ts": time.monotonic(),
        }
        with self._gps_lock:
            self._gps = fix

    def read_gps(self) -> dict | None:
        with self._gps_lock:
            fix = dict(self._gps) if self._gps else None
        if fix is not None and time.monotonic() - fix["ts"] > MAX_FIX_AGE_S:
            return None  # events stopped arriving; treat as no fix
        return fix

    @property
    def available(self) -> dict[str, bool]:
        return {
            "accelerometer": self.accelerometer is not None,
            "gyroscope": self.gyrometer is not None,
            "compass": self.compass is not None,
            "inclinometer": self.inclinometer is not None,
            "gps": self.gps_ready,
        }
