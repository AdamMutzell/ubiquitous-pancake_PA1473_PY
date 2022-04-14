#!/usr/bin/env pybricks-micropython
import sys
import __init__

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
# "ColorSensor": [one Color Sensor] for measuring line colors to follow the line .
# "TouchSensor": [one Touch Sensor] for detecting a pallet on the forks" .
# "UltrasonicSensor": [one Ultrasonic Sensor] for detection of obstacles.


EV3 = EV3Brick()

Left_drive = Port.C
Right_drive = Port.B
Crane_motor = Port.A
Front_button = Port.S1
Light_port = Port.S3
Ultrasonic_sensor = Port.S4

# Initialze the drivebase of the robot. Handles the motors (USE THIS)
# May need to change wheel_diameter and axel_track
TRUCK = DriveBase(left_motor=Motor(Left_drive, gears=[12, 20]), right_motor=Motor(Right_drive, gears=[12, 20]),
                  wheel_diameter=47, axle_track=128)


# Measure of reflection:
WHAITE = 100
BLACK = 15
THRESHOLD = (WHAITE + BLACK) / 2

# Speed:
DRIVING_INITAL = 50

# Drive on the line:


def main():  # Main Class
    crane_up(Crane_motor)
    drive()
    crane_hold(Crane_motor)
    drive()

    # crane_pickup(Crane_motor, Front_button, TRUCK,
    #             0, max_angle=180, min_angle=0)

    return 0


def drive():
    Light_sensor = ColorSensor(Light_port)
    drive_check = True
    while drive_check is True:
        TRUCK.drive(-DRIVING_INITAL, Light_sensor.reflection()-THRESHOLD)
    return


def button_pressed(button_port):  # Function for detecting button press
    Front_button = TouchSensor(button_port)
    if Front_button.pressed():
        return True
    else:
        return False


# Function for detecting obstacles and stopping the robot.
def obstacle(accepted_distance, current_mode, sensor_port):
    sensor = UltrasonicSensor(sensor_port)
    distance = sensor.distance()  # Value in mm
    if distance < accepted_distance and current_mode == "Driving":
        return False
    return True  # Inert message


# Function for detecting if a pickup of an item has failed
def detect_item_fail(pickupstatus, button_port):
    if pickupstatus == True:
        return button_pressed(button_port)
    else:
        return False


def crane_up(crane_port):  # Function for moving the crane up
    """
    Crane_port - Class contatning the port, containing the port of the crane

    Returns an angle of the crane at it's maximum angle
    """
    speed_of_crane = 100
    Crane_motor = Motor(crane_port)
    return Crane_motor.run_until_stalled(speed_of_crane, duty_limit=90)


def crane_hold(crane_port):  # Function for moving the crane up
    """
    Crane_port - Class contatning the port, containing the port of the crane

    Returns an angle of the crane at it's maximum angle
    """
    speed_of_crane = 50
    Crane_motor = Motor(crane_port)
    Crane_motor.run_target(speed_of_crane, 200)
    Crane_motor.hold()


def crane_down(crane_port):  # Function for moving the crane down
    """
    Crane_port - Class contatning the port, containing the port of the crane

    Returns an angle of the crane at it's minimum angle
    """
    speed_of_crane = -100
    Crane_motor = Motor(crane_port)
    return Crane_motor.run_until_stalled(speed_of_crane, duty_limit=90)

# Function for moving the crane up


def crane_pickup(crane_port, touch_port, DriveBase, angle_of_crane, max_angle, min_angle):
    """
    Crane_port - Class contatning the port, containing the port of the crane
    DriveBase - Class that handles the drving of the robot
    angle_of_crane - Int, containing the angle the crane should be
    max_angle - int, containing the maximum angle the crane can be
    min_angle - int, containing the minimum angle the crane can be

    Returns null
    """
    # Initializing the variables
    speed_of_crane = 100
    raise_angle = 50
    Crane_motor = Motor(crane_port)
    distance_traveled = 0
    ROBOT = DriveBase

    # Makes sure that the angle of the crane is valid
    if angle_of_crane < min_angle:
        angle_of_crane = min_angle

    # Raise the crane to the angle of the pallet
    Crane_motor.run_target(speed_of_crane, angle_of_crane, )

    # Drive forward for 100mm
    while button_pressed(touch_port) is False:
        ROBOT.straight(-100)
        distance_traveled += 100

    # Raise the crane slightly to hold the planet
    if (angle_of_crane + raise_angle) <= max_angle:
        Crane_motor.run_target(speed_of_crane, angle_of_crane + raise_angle)
    else:
        Crane_motor.run_target(speed_of_crane, max_angle)

    # Drive back
    ROBOT.straight(distance_traveled)


if __name__ == '__main__':  # Keep this!
    sys.exit(main())
