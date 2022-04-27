#!/usr/bin/env pybricks-micropython
from main import drive, crane_movement, crane_pickup


def main():  # Main Class
    # Testing the crane
    # drive()
    pickupstatus = True
    max_angle = crane_movement(Crane_motor, 1, 50)
    min_angle = crane_movement(Crane_motor, -1, 50)

    crane_pickup(Crane_motor, TRUCK, Front_button, -1000, max_angle, min_angle)

    detect_item_fail(pickupstatus, Front_button)


if __name__ == '__main__':  # Keep this!
    sys.exit(main())
