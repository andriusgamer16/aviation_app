# PyAvionics micro:bit sensor pod.
#
# Flash from the project root with the micro:bit plugged in over USB:
#   .\.venv\Scripts\python -m uflash firmware\microbit_main.py
#
# Streams "MB,<ax>,<ay>,<az>,<mx>,<my>,<mz>,<btnB>" at ~25 Hz over USB
# serial (115200 baud). ax/ay/az are milli-g; mx/my/mz are the raw
# magnetometer field in nano-tesla. There is no on-device compass
# calibration ("tilt to fill the screen") — the app computes a
# tilt-compensated heading and learns the calibration by itself as the
# board is rotated.
#
# Button B: ask the app to re-level (same as the LEVEL button in the UI).
from microbit import *

display.show(Image.ARROW_N)

while True:
    print("MB,%d,%d,%d,%d,%d,%d,%d" % (
        accelerometer.get_x(),
        accelerometer.get_y(),
        accelerometer.get_z(),
        compass.get_x(),
        compass.get_y(),
        compass.get_z(),
        1 if button_b.was_pressed() else 0,
    ))
    sleep(40)
