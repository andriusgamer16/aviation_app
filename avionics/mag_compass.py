"""Tilt-compensated magnetic heading from raw magnetometer samples.

The micro:bit's on-device compass.heading() requires the blocking LED
calibration game on every power-up and assumes the board is flat. This
module instead consumes raw field samples (nano-tesla, device frame):

- Hard-iron offsets are estimated from the min/max envelope that
  accumulates as the board gets rotated in normal use, tracked in the
  device frame (a property of the board, so it survives re-mounting and
  LEVEL), and persisted to disk.
- Headings are produced only once at least two axes have seen enough
  span to trust the envelope (a yaw rotation sweeps two device axes for
  any static mounting).
- Tilt is compensated with the instantaneous gravity vector, so the
  heading refers to the aircraft nose in the re-leveled frame.
"""
from __future__ import annotations

import json
import logging
import math
from pathlib import Path

from .fusion import MountCalibration, Vec3

log = logging.getLogger("avionics.magcompass")

# Horizontal Earth field is ~15-20 uT in Europe, so a full flat turn sweeps
# a ~30-40 uT (30000-40000 nT) span on each horizontal axis.
SPAN_REQ_NT = 25000.0

_DEFAULT_STORE = Path(__file__).resolve().parent.parent / "mag_cal.json"


class MagCompass:
    def __init__(self, store: Path | None = _DEFAULT_STORE) -> None:
        self._min = [math.inf, math.inf, math.inf]
        self._max = [-math.inf, -math.inf, -math.inf]
        self._store = store
        self._saved_progress = 0.0
        if store is not None and store.exists():
            try:
                data = json.loads(store.read_text())
                lo, hi = data["min"], data["max"]
                if len(lo) == 3 and len(hi) == 3 and all(
                    isinstance(v, (int, float)) and math.isfinite(v)
                    for v in lo + hi
                ):
                    self._min, self._max = list(lo), list(hi)
                    self._saved_progress = self.progress
                    log.info("magnetometer calibration loaded (progress %.0f%%)",
                             self.progress * 100)
            except Exception as exc:
                log.warning("could not load %s: %s", store, exc)

    @property
    def spans(self) -> list[float]:
        return [
            (hi - lo) if hi >= lo else 0.0
            for lo, hi in zip(self._min, self._max)
        ]

    @property
    def progress(self) -> float:
        """0..1: second-largest axis span vs the required span."""
        second = sorted(self.spans)[1]
        return min(1.0, second / SPAN_REQ_NT)

    @property
    def ready(self) -> bool:
        return self.progress >= 1.0

    def observe(self, m: Vec3) -> None:
        changed = False
        for i, v in enumerate(m):
            if not math.isfinite(v):
                return
            if v < self._min[i]:
                self._min[i] = v
                changed = True
            if v > self._max[i]:
                self._max[i] = v
                changed = True
        if changed and self._store is not None:
            p = self.progress
            if p - self._saved_progress >= 0.05 or (p >= 1.0 > self._saved_progress):
                self._save(p)

    def _save(self, progress: float) -> None:
        try:
            self._store.write_text(
                json.dumps({"min": self._min, "max": self._max})
            )
            self._saved_progress = progress
        except OSError as exc:
            log.warning("could not save %s: %s", self._store, exc)

    def corrected(self, m_dev: Vec3) -> Vec3:
        """Field with the current hard-iron offset estimate removed."""
        return tuple(
            v - (lo + hi) / 2.0 if hi >= lo else v
            for v, lo, hi in zip(m_dev, self._min, self._max)
        )

    def heading(
        self,
        m_dev: Vec3,
        g_dev: Vec3,
        mount: MountCalibration,
    ) -> float | None:
        """Magnetic heading of the nose (deg, 0=north) or None if not ready."""
        if not self.ready:
            return None
        mv = mount.apply(self.corrected(m_dev))
        gv = mount.apply(g_dev)
        tilt = MountCalibration()
        if not tilt.capture(gv):
            return None
        mh = tilt.apply(mv)
        # Nose is +Y in the re-leveled frame; clockwise-positive heading.
        return math.degrees(math.atan2(-mh[0], mh[1])) % 360.0

    def reset(self) -> None:
        """Discard the learned envelope (and its file) to recalibrate."""
        self._min = [math.inf, math.inf, math.inf]
        self._max = [-math.inf, -math.inf, -math.inf]
        self._saved_progress = 0.0
        if self._store is not None:
            try:
                self._store.unlink(missing_ok=True)
            except OSError as exc:
                log.warning("could not delete %s: %s", self._store, exc)
        log.info("magnetometer calibration reset")

    def debug(self) -> dict:
        def fin(v: float) -> float | None:
            return v if math.isfinite(v) else None

        spans = self.spans
        return {
            "min": [fin(v) for v in self._min],
            "max": [fin(v) for v in self._max],
            "offset": [
                fin((lo + hi) / 2.0) if hi >= lo else None
                for lo, hi in zip(self._min, self._max)
            ],
            "span": spans,
            "progress": round(self.progress, 3),
            "ready": self.ready,
            "spanRequiredNt": SPAN_REQ_NT,
        }
