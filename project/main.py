#!/usr/bin/env pybricks-micropython
import sys
import __init__
from pybrick.hubs import EV3Hub
from pybrick.ev3devices import Motor
from pybrick.parameters import Port

EV3 = EV3Hub()

Left_drive = Port.C
Right_drive = Port.B
Crane_motor = Port.A
Front_button = Port.S1
Light_sensor = Port.S3
Ultrasonic_sensor = Port.S4


def main():  # Main Class
    return 0


# Good bye
# Hej
if __name__ == '__main__':  # Keep this!
    sys.exit(main())
