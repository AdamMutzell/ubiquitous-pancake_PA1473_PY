#!/usr/bin/env pybricks-micropython
import sys
import __init__

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.pupdevices import ColorSensor, ForceSensor, UltrasonicSensor, TouchSensor
# "ColorSensor": [one Color Sensor] for measuring line colors to follow the line .
# "ForceSensor": [one Touch Sensor] for detecting a pallet on the forks" .
# "UltrasonicSensor": [one Ultrasonic Sensor] for detection of obstacles.


EV3 = EV3Brick()

Left_drive = Port.C
Right_drive = Port.B
Crane_motor = Port.A
Front_button = Port.S1
Light_sensor = Port.S3
Ultrasonic_sensor = Port.S4

# Initialze the drivebase of the robot. Handles the motors (USE THIS)
# May need to change wheel_diameter and axel_track
TRUCK = DriveBase(left_motor=Left_drive, right_motor=Right_drive,
                  wheel_diameter=56, axle_track=118)


def main():  # Main Class
    return 0


def button_pressed(port):  # Function for detecting button press
    Front_button = TouchSensor(port)
    if Front_button.pressed():
        return True
    else:
        return False


def crane_up(crane_port):  # Function for moving the crane up
    """
    Crane_port - Class contatning the port, containing the port of the crane

    Returns an angle of the crane at it's maximum angle
    """
    speed_of_crane = 50
    Crane_motor = Motor(crane_port)
    return Crane_motor.run_until_stalled(speed_of_crane, duty_limit=90)


def crane_down(crane_port):  # Function for moving the crane down
    """
    Crane_port - Class contatning the port, containing the port of the crane

    Returns an angle of the crane at it's minimum angle
    """
    speed_of_crane = -50
    Crane_motor = Motor(crane_port)
    return Crane_motor.run_until_stalled(speed_of_crane, duty_limit=90)


# Function for moving the crane up
def crane_pickup(crane_port, DriveBase, angle_of_crane, max_angle, min_angle):
    """
    Crane_port - Class contatning the port, containing the port of the crane
    Motor_ports - Tuple, containing the ports of the motors
    angle_of_crane - Int, containing the angle the crane should be
    max_angle - int, containing the maximum angle the crane can be
    min_angle - int, containing the minimum angle the crane can be

    Returns null
    """
    # Initializing the variables
    speed_of_crane = 50
    raise_angle = 5
    Crane_motor = Motor(crane_port)
    ROBOT = DriveBase

    # Makes sure that the angle of the crane is valid
    if angle_of_crane < min_angle:
        angle_of_crane = min_angle

    # Raise the crane to the angle of the pallet
    Crane_motor.run_target(speed_of_crane, angle_of_crane, )

    # Drive forward for 100mm
    while button_pressed() is False:
        ROBOT.straight(100)
        distance_traveled -= 100

    # Raise the crane slightly to hold the planet
    if (angle_of_crane + raise_angle) <= max_angle:
        Crane_motor.run_target(speed_of_crane, angle_of_crane + raise_angle)
    else:
        Crane_motor.run_target(speed_of_crane, max_angle)

    # Drive back
    ROBOT.straight(distance_traveled)


if __name__ == '__main__':  # Keep this!
    sys.exit(main())
