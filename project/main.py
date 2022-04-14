#!/usr/bin/env pybricks-micropython
from pickle import TRUE
import sys
import __init__

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.parameters import Port, Color,Direction
from pybricks.robotics import DriveBase
from pybricks.pupdevices import ColorSensor, UltrasonicSensor
from pybricks.tools import wait
# "ColorSensor": [one Color Sensor] for measuring line colors to follow the line .
# "TouchSensor": [one Touch Sensor] for detecting a pallet on the forks" .
# "UltrasonicSensor": [one Ultrasonic Sensor] for detection of obstacles.


GREEN = Color(h=120, s=100, v=100)
BLUE = Color(h=240, s=100, v=100)
RED = Color(h=359, s=97, v=39)
BROWN = Color(h=17, s=48, v=15)
YELLOW = Color(h=60, s=100, v=100)

colours = [GREEN, BLUE, RED, BROWN, YELLOW]



EV3 = EV3Brick()
Crane_motor = Motor(Port.A)
Right_drive = Motor(Port.B,positive_direction=Direction.COUNTERCLOCKWISE,gears=[12,20])
Left_drive = Motor(Port.C,positive_direction=Direction.COUNTERCLOCKWISE,gears=[12,20])

Front_button = TouchSensor(Port.S1)
Light_sensor = ColorSensor(Port.S3)
Ultrasonic_sensor = UltrasonicSensor(Port.S4)

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
    crane_movement(Crane_motor, 1, 50)
    crane_movement(Crane_motor, -1, 50)
    return None


def drive():
    drive_check = True
    while drive_check is True:
        TRUCK.drive(DRIVING_INITAL, Light_sensor.reflection()-THRESHOLD)
    return None


def button_pressed(button_port):  # Function for detecting button press
    #Front_button = TouchSensor(button_port)
    if Front_button.pressed():
        return True
    else:
        return False


def obstacle(accepted_distance, current_mode, sensor): # Function for detecting obstacles and stopping the robot.
    distance = sensor.distance() #Value in mm
    while distance < accepted_distance and current_mode == "Driving":
        wait(100)
        distance = sensor.value()
    return None #Inert message


def detect_item_fail(pickupstatus, button): # Function for detecting if a pickup of an item has failed
    if pickupstatus == TRUE:
        return button_pressed(button)
    else:
        return TRUE


def crane_movement(crane_port, direction, speed):  # Function for moving the crane up
    """
    Crane_port - Class contatning the port, containing the port of the crane

    Returns an angle of the crane at it's maximum angle
    """
    speed_of_crane = speed * direction
    Crane_motor = Motor(crane_port)
    return Crane_motor.run_until_stalled(speed_of_crane, duty_limit=90)


def crane_hold(crane_port):  # Function for moving the crane up
    """
    Crane_port - Class contatning the port, containing the port of the crane

    Returns an angle of the crane at it's maximum angle
    """
    speed_of_crane = 50
    Crane_motor = Motor(crane_port)
    return Crane_motor.run_until_stalled(speed_of_crane, duty_limit=90)

# Function for moving the crane up
def crane_pickup(crane_port, DriveBase, angle_of_crane, max_angle, min_angle):
    """
    Crane_port - Class contatning the port, containing the port of the crane
    DriveBase - Class that handles the drving of the robot
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

    return angle_of_crane

def get_colour():
    return Light_sensor.color()
    
colours = [Color()]


def set_area(colours,current_area):
    threshold = 15
    current_colour = get_colour()
    for colour in colours:
        if get_colour() == colour:
            if colour != current_area:
                current_area = colour
                #area has been switched





if __name__ == '__main__':  # Keep this!
    sys.exit(main())