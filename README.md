# PyAvionics

A lightweight, Garmin-style avionics display (PFD) that runs in the browser,
driven by a Python backend reading this computer's real sensors:

| Instrument | Sensor used | Fallbacks (in order) |
|---|---|---|
| Attitude (pitch/roll) | Inclinometer (Windows fused), else accelerometer + gyroscope | micro:bit accelerometer, simulator |
| Heading (HSI) | Compass (magnetic + true when available) | micro:bit compass, simulator |
| G-load, slip/skid | Accelerometer | micro:bit accelerometer, simulator |
| Turn rate | Heading derivative (mount-independent) | simulator |
| Position, ground speed, track, altitude | Windows Location / GPS | simulator |

Sensors are accessed through WinRT (`Windows.Devices.Sensors`,
`Windows.Devices.Geolocation`) via the `winrt-*` PyPI packages. Any sensor
the machine lacks is transparently replaced by a built-in flight simulator,
and the display annunciates exactly which sources are live vs simulated
(amber `SIM` tags, mode chip in the top-right).

## Run

```powershell
python -m venv .venv
.\.venv\Scripts\pip install -r requirements.txt
.\.venv\Scripts\python server.py            # auto: real sensors where present
.\.venv\Scripts\python server.py --sim      # force full simulation
```

Open http://127.0.0.1:8000 — use `--host 0.0.0.0` to view it from a tablet
on the same network.

### BBC micro:bit as a sensor pod

Desktops have no motion sensors, but a BBC micro:bit (v1 or v2) over USB
provides a real accelerometer + compass. One-time flash (with the board
plugged in):

```powershell
.\.venv\Scripts\python -m uflash firmware\microbit_main.py
```

The backend watches for the board continuously — plug it in (even while
the server runs) and attitude/heading/g-load go live with green `MICROBIT`
annunciators.

**Compass**: no on-device calibration game needed. The board streams its
raw magnetometer and the backend computes a tilt-compensated heading,
learning the hard-iron offsets automatically as you rotate the board —
just turn it through a slow full circle (any mounting). The `HDG`
annunciator shows `CAL n% — ROTATE BOARD` until headings are trustworthy,
then goes live; the learned calibration persists in `mag_cal.json` across
restarts. (Button **A** still runs the classic on-device calibration,
used as a fallback.) Button **B** re-levels the attitude, same as the
LEVEL button in the UI.

If the board doesn't appear: use a *data* USB cable (charge-only cables
are a classic trap) — Windows should show a `MICROBIT` drive and a COM
port. If pitch responds inverted on your unit, set
`AVIONICS_MB_Y_SIGN=1` before starting the server.

### Notes

- **Location permission**: for GPS/position, enable *Windows Settings >
  Privacy & security > Location*. Desktops without GNSS hardware still get
  a Wi-Fi-based fix (no speed/track — those show `---`).
- **LEVEL button**: mounts vary (flat on a desk, propped upright like a
  panel), so press **LEVEL** to capture the current gravity direction as
  straight-and-level; it is applied as a proper rotation, valid for any
  static mounting.
- Altitude and speed are GPS-derived (no pitot-static), hence the `GS` /
  `GPS` labels on the tapes.

## Architecture

```
server.py                FastAPI + uvicorn; samples sensors at 20 Hz and
                         broadcasts JSON frames over /ws to every browser
avionics/
  windows_sensors.py     WinRT sensor + Geolocator access (guarded imports)
  microbit_sensors.py    micro:bit serial reader (hot-plug, auto-reconnect)
  fusion.py              complementary filter (accel+gyro) when the OS has
                         no fused Inclinometer; rotation-based mount
                         calibration (LEVEL)
  simulator.py           point-mass aircraft flying gentle maneuvers
  manager.py             merges real + simulated sources into one frame,
                         computes turn rate / vertical speed
firmware/
  microbit_main.py       MicroPython streamer for the micro:bit pod
static/
  index.html, style.css
  pfd.js                 canvas PFD: attitude, speed/alt tapes, VSI, HSI,
                         annunciators; 60 fps with smoothing + reconnect
```

### HTTP/WS API

- `GET /` — the PFD
- `WS /ws` — 20 Hz JSON frames:
  `{t, att{pitch,roll,src}, hdg{mag,true,src}, acc{x,y,z,g,slip,src},
    gyro{x,y,z,src}|null, turnRateDps, vsMps,
    gps{lat,lon,altM,gsMps,trkDeg,accM,fixSrc,ageS,src},
    status{mode,sensors,zeroed,mbMagCal}}`
- `POST /api/zero` — capture current attitude as level
- `GET /api/status` — sensor availability and mode
