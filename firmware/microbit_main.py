# PyAvionics micro:bit sensor pod.
#
# Flash from the project root with the micro:bit plugged in over USB:
#   .\.venv\Scripts\python -m uflash firmware\microbit_main.py
#
# Streams "MB,<ax>,<ay>,<az>,<mx>,<my>,<mz>,<heading>,<btnB>" at ~25 Hz
# over USB serial (115200 baud). ax/ay/az are milli-g; mx/my/mz are the
# raw magnetometer field in nano-tesla (no calibration needed — the app
# computes a tilt-compensated heading and auto-calibrates as the board is
# rotated). heading is the on-device value, 0-359 once the on-device
# compass is calibrated, else -1 (kept as a fallback).
#
# Button A: start the on-device compass-calibration game (optional).
# Button B: ask the app to re-level (same as the LEVEL button in the UI).
from microbit import *

display.show(Image.ARROW_N)

while True:
    if button_a.was_pressed():
        compass.calibrate()
        display.show(Image.ARROW_N)
    if compass.is_calibrated():
        hdg = compass.heading()
    else:
        hdg = -1
    print("MB,%d,%d,%d,%d,%d,%d,%d,%d" % (
        accelerometer.get_x(),
        accelerometer.get_y(),
        accelerometer.get_z(),
        compass.get_x(),
        compass.get_y(),
        compass.get_z(),
        hdg,
        1 if button_b.was_pressed() else 0,
    ))
    sleep(40)
