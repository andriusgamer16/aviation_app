# PyAvionics micro:bit sensor pod.
#
# Flash from the project root with the micro:bit plugged in over USB:
#   .\.venv\Scripts\python -m uflash firmware\microbit_main.py
#
# Streams "MB,<ax>,<ay>,<az>,<mx>,<my>,<mz>,<a1>,<btnB>" at ~25 Hz over
# USB serial (115200 baud):
#   ax/ay/az  accelerometer, milli-g
#   mx/my/mz  raw magnetometer field, nano-tesla (heading is computed and
#             auto-calibrated by the app; no "tilt to fill the screen")
#   a1        analog reading of pin P1, 0-1023 (= 0-3.3 V) — airspeed
#             from a DC-motor + propeller used as a wind generator
#   btnB      1 on a B-button press (asks the app to re-level)
#
# Airspeed motor wiring: one motor lead to GND, the other through a
# ~1 kOhm series resistor to P1. A spinning motor can generate more than
# 3.3 V (and negative voltage when spun backwards) — never feed the pin
# beyond 0-3.3 V.
from microbit import *

display.show(Image.ARROW_N)

while True:
    print("MB,%d,%d,%d,%d,%d,%d,%d,%d" % (
        accelerometer.get_x(),
        accelerometer.get_y(),
        accelerometer.get_z(),
        compass.get_x(),
        compass.get_y(),
        compass.get_z(),
        pin1.read_analog(),
        1 if button_b.was_pressed() else 0,
    ))
    sleep(40)
