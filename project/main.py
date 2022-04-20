#!/usr/bin/env pybricks-micropython
import sys
import __init__
import Colour_Calibrator

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import SoundFile
# To do:
# Fix the crane pickup function for elevated surfaces
# Might be able to show that elevated surface by taking half of max_angle.
# Colours value for paths
# Emergency mode

# green = Color(h=120, s=100, v=100)
# blue = Color(h=240, s=100, v=100)
# red = Color(h=359, s=97, v=39)
# brown = Color(h=17, s=48, v=15)
# yellow = Color(h=60, s=100, v=100)
EV3 = EV3Brick()

Crane_motor = Motor(Port.A, gears=[12, 36])
Right_drive = Motor(
    Port.B, positive_direction=Direction.COUNTERCLOCKWISE, gears=[12, 20])
Left_drive = Motor(
    Port.C, positive_direction=Direction.COUNTERCLOCKWISE, gears=[12, 20])

Front_button = TouchSensor(Port.S1)
Light_sensor = ColorSensor(Port.S3)
Ultrasonic_sensor = UltrasonicSensor(Port.S4)


saved_colours = open("savedColours.txt", "r")
colours = {"Zone_1": Color.GREEN, "Zone_2": Color.BLUE,
           "Zone_3": Color.RED, "Roundabout": Color.BROWN, "Warehouse": Color.YELLOW}
colours = Colour_Calibrator.Calibrate_Colours(colours, Light_sensor)
current_colour = Color.WHITE


# Initialze the drivebase of the robot. Handles the motors (USE THIS)
# May need to change wheel_diameter and axel_track
TRUCK = DriveBase(left_motor=Right_drive, right_motor=Left_drive,
                  wheel_diameter=47, axle_track=128)


# Measure of reflection:
def THRESHOLD():
    WHITE = 100
    BLACK = 15

    THRESHOLD_color = (WHITE+BLACK) / 2
    """
    if Light_sensor == Color.GREEN:
        THRESHOLD_color = (WHITE + Color.GREEN) / 2
    if Light_sensor == Color.BLUE:
        THRESHOLD_color = (WHITE + Color.BLUE) / 2
    if Light_sensor == Color.RED:
        THRESHOLD_color = (WHITE + Color.RED) / 2
    if Light_sensor == Color.BROWN:
        THRESHOLD_color = (WHITE + Color.BROWN) / 2
    if Light_sensor == Color.YELLOW:
        THRESHOLD_color = (WHITE + Color.YELLOW) / 2
    """
# Robot should be stop when it black, because the warehouses have black line. !!!!!!!!
    # if Light_sensor == Color.BLACK:
    #    THRESHOLD_color = TRUCK.stop()
    #    print(sound_start)
    return THRESHOLD_color


# sounds and notification:
# I need to know how can I import sound file. ??????
sound_start = EV3.speaker.beep()
#sound_GREEN = EV3.speaker.GREEN()
#sound_BLUE = EV3.speaker.BLUE()
#sound_RED = EV3.speaker.RED()
#sound_BROWN = EV3.speaker.BROWN()
#sound_YELLOW = EV3.speaker.YELLOW()
#sound_BLACK = EV3.speaker.BLACK()


# Speed:
DRIVING_INITAL = 50

# Drive on the line:


def main():  # Main Class
    # Testing the crane
    # drive()
    pickupstatus = True

    Crane_motor.reset_angle(0)
    max_angle = crane_movement(Crane_motor, 1, 50)
    min_angle = crane_movement(Crane_motor, -1, 50)

    crane_pickup(Crane_motor, TRUCK, Front_button, -1000, max_angle, min_angle)

    detect_item_fail(pickupstatus, Front_button)


def drive():
    drive_check = True
    pickupstatus = False
    while drive_check is True:
        if obstacle(300, "Driving", Ultrasonic_sensor) is True:
            TRUCK.stop()

        print(sound_start)
        TRUCK.drive(DRIVING_INITAL, Light_sensor.reflection()-THRESHOLD())
    return None


def button_pressed(Front_button):  # Function for detecting button press
    """
    Front_button, class handling the front button of the robot

    Returns true if the button is pressed, false otherwise
    """

    if Front_button.pressed():
        return True
    else:
        return False


# Function for detecting obstacles and stopping the robot.
def obstacle(accepted_distance, current_mode, sensor):
    """
    accepted_distance - int, the distance to not accept any obstacles
    current_mode - str, the mode of the robot
    sensor - Class, handling the ultra sonic sensor of the robot

    returns true if an obstacle is detected, false otherwise
    """
    distance = sensor.distance()  # Value in mm
    if distance < accepted_distance and current_mode == "Driving":
        return True
    return False


# Function for detecting if a pickup of an item has failed
def detect_item_fail(pickupstatus, button):
    """
    pickupstatus - boolean, True if the truck is currently picking up an item
    button, a class handling the front button of the robot

    Returns True if the pickup has failed, False otherwise
    """
    if pickupstatus == True:
        EV3.speaker.beep()
        return button_pressed(button)
    else:
        return True


def crane_movement(Crane_motor, direction, speed):  # Function for moving the crane up
    """
    Crane_port - Class contatning the port, containing the port of the crane
    direction, a value between -1 and 1, indicating the direction of the movement
    speed, a value between 0 and 100, indicating the speed of the movement
    Returns an angle of the crane at it's maximum angle
    """
    speed_of_crane = speed * direction
    return Crane_motor.run_until_stalled(speed_of_crane, duty_limit=75)


def crane_hold(Crane_motor):  # Function for moving the crane up
    """
    Crane_port - Class contatning the port, containing the port of the crane

    Returns an angle of the crane at it's maximum angle
    """
    speed_of_crane = 50
    return Crane_motor.run_until_stalled(speed_of_crane, duty_limit=90)

# Function for moving the crane up


def crane_pickup(Crane_motor, DriveBase, Front_button, angle_of_crane, max_angle, min_angle):
    """
    Crane_port - Class containing the port, containing the port of the crane
    DriveBase - Class that handles the drving of the robot
    Front_button - Class that handles the button on the front of the robot
    angle_of_crane - Int, containing the angle the crane should be
    max_angle - int, containing the maximum angle the crane can be
    min_angle - int, containing the minimum angle the crane can be

    Returns None
    """
    # To do, check out stop function on target and stall limits

    # Initializing the variables
    speed_of_crane = 50
    raise_angle = 50
    distance_traveled = 0
    ROBOT = DriveBase

    # Seetings for the crane motor
    Crane_motor.control.stall_tolerances(stall_limit=90, stall_time_limit=5000)

    # Makes sure that the angle of the crane is valid
    if angle_of_crane < min_angle:
        angle_of_crane = min_angle

    # Raise the crane to the angle of the pallet
    Crane_motor.run_target(speed_of_crane, angle_of_crane, )

    # Drive forward for 100mm
    while button_pressed(Front_button) is False:
        print(button_pressed(Front_button))

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
    """
    Returns the colour of the ground the robot is looking at
    """
    return Light_sensor.color()


def get_area(colours, current_area):
    threshold = 15
    current_colour = get_colour()
    for colour in colours:
        if get_colour() == colour:
            if colour != current_area:
                current_area = colour
                # area has been switched
    return current_area


def exit_zone(initial_zone):
    TRUCK.turn(180)
    if initial_zone != get_area():
        # robot has left the zone
        return True
    else:
        return False


if __name__ == '__main__':  # Keep this!
    sys.exit(main())
