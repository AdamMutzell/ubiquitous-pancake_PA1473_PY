#!/usr/bin/env pybricks-micropython
import sys
import __init__
from Colour_follower import angle_to_colour, colour_target, rgb_to_hsv
import math
import time
import Colour_Manager
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color, Direction, Button, Stop
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
light_sensor = ColorSensor(Port.S3)
Ultrasonic_sensor = UltrasonicSensor(Port.S4)


#saved_colours = open("savedColours.txt", "r")

colours = {"Zone_1": Color.GREEN, "Zone_2": Color.BLUE,
           "Zone_3": Color.RED, "Roundabout": Color.BROWN, "Warehouse": Color.YELLOW}

use_calibrator = False
going_to_target = False
# Change to false to skip calibration mode and use .txt file if avalible

if use_calibrator:
    colours = Colour_Manager.Calibrate_Colours(colours, light_sensor)
    print("colours calibrated as :"+str(colours))
# else:
#   colours = Colour_Manager.Get_File()
    # ^This does not work yet^

target_zone = Color.WHITE
final_target_zone = colours["Zone_2"]  # set this using user input?

# Initialze the drivebase of the robot. Handles the motors (USE THIS)
# May need to change wheel_diameter and axel_track
TRUCK = DriveBase(left_motor=Right_drive, right_motor=Left_drive,
                  wheel_diameter=47, axle_track=128)
sound_start = EV3.speaker.beep()


# Speed:
DRIVING_INITAL = 50

# Drive on the line:

btn = Button

# START


def startup():
    btn.wait_for_bump(['up', 'left', 'right'], 2000)
    if btn.up:
        # kör igång calibrering
        EV3Brick.screen.print('Calibration start')
        return 0
    elif btn.left:
        # drive towards red warehouse
        EV3Brick.screen.print('Driving towards Red Warehouse')
        return [Color.GREEN, Color.BROWN, Color.RED, Color.YELLOW]
    elif btn.right:
        # drive towards blue warehouse
        EV3Brick.screen.print('Driving towards Blue Warehouse')
        return [Color.GREEN, Color.BROWN, Color.BLUE, Color.YELLOW]


def main():  # Main Class
    pickup_on = True
    start_angle = crane_movement(Crane_motor, 1, 50)
    wait(5000)

    while pickup_on == True:
        wait(2000)
        print(Crane_motor.angle())
        pickup_on = emergency_mode(start_angle, Crane_motor)
    print("tappade :(")


def test_drive():
    drive()


def test_crane():
    Crane_motor.reset_angle(0)
    max_angle = crane_movement(Crane_motor, 1, 50)
    min_angle = crane_movement(Crane_motor, -1, 50)

    crane_pickup(Crane_motor, TRUCK, Front_button, -1000, max_angle, min_angle)
    crane_pickup(Crane_motor, TRUCK, Front_button,
                 max_angle/2, max_angle, min_angle)


def drive():
    drive_check = True
    pickupstatus = False
    colour_one = [438, 47, 21]
    colour_two = [137, 80, 26]
    line_to_follow = colour_target(colour_one, colour_two)
    color_rgb = light_sensor.rgb()
    color_hsv = rgb_to_hsv(color_rgb[0], color_rgb[1], color_rgb[2])
    list_of_colours = []
    index_of_colours = 0

    while drive_check is True:
        # Check the line it's following
        #colour_two = list_of_colours[index_of_colours]

        # Check the line to follow
        line_to_follow = colour_target(colour_one, colour_two)
        color_rgb = light_sensor.rgb()
        color_hsv = rgb_to_hsv(color_rgb[0], color_rgb[1], color_rgb[2])

        # Check if the next colour is present
        # if light_sensor.color == list_of_colours[index_of_colours + 1]:
        #    index_of_colours += 1
        #    colour_two = list_of_colours[index_of_colours]

        if obstacle(300, "Driving", Ultrasonic_sensor) is True:
            TRUCK.stop()

        # print(sound_start)
        TRUCK.drive(DRIVING_INITAL, angle_to_colour(line_to_follow, color_hsv))
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
# Might be worth adding a check for the duty limit of a crane
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
    Crane_motor.stop()

    speed_of_crane = speed * direction
    return Crane_motor.run_until_stalled(speed_of_crane, then=Stop.BRAKE, duty_limit=50)


def crane_hold(Crane_motor):  # Function for moving the crane up
    """
    Crane_port - Class contatning the port, containing the port of the crane

    Returns an angle of the crane at it's maximum angle
    """
    # To prevent problems with the crane holding
    Crane_motor.stop()

    speed_of_crane = 50
    return Crane_motor.run_until_stalled(speed_of_crane, Stop.HOLD, duty_limit=90)

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


def Set_Target():
    current_zone = Colour_Manager.get_area()
    if current_zone == final_target_zone:
        # go towards warehouse
        pass
    elif current_zone == colours["Roundabout"]:
        target_zone = final_target_zone
    else:
        target_zone = colours["Roundabout"]


def Siren(beep_frequency, sine_frequency):
    """call this inside a while loop for desired effect"""
    threshold = 0.8
    sine_wave = abs(math.sin(time.time()*sine_frequency))
    if sine_wave >= threshold:
        EV3.speaker.beep(beep_frequency)


def exit_zone(initial_zone):
    TRUCK.turn(180)
    if initial_zone != Colour_Manager.get_area():
        # robot has left the zone
        return True
    else:
        return False
    # Very bad code! Please ignore


def emergency_mode(angle, crane_motor):
    if angle + 5 < crane_motor.angle():
        return False
    else:
        return True


if __name__ == '__main__':  # Keep this!
    sys.exit(main())
