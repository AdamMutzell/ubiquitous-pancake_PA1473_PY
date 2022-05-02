#!/usr/bin/env pybricks-micropython
import sys
from tkinter import E
from turtle import distance
import __init__
from Colour_follower import angle_to_colour, colour_target, rgb_to_hsv
from Crane_Manager import crane_movement, crane_pickup
from Sensor_Manager import button_pressed, obstacle, detect_item_fail
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


EV3 = EV3Brick()

# Initialzie the components of the robot
Crane_motor = Motor(Port.A, gears=[12, 36])
Right_drive = Motor(
    Port.B, positive_direction=Direction.COUNTERCLOCKWISE, gears=[12, 20])
Left_drive = Motor(
    Port.C, positive_direction=Direction.COUNTERCLOCKWISE, gears=[12, 20])

Front_button = TouchSensor(Port.S1)
light_sensor = ColorSensor(Port.S3)
Ultrasonic_sensor = UltrasonicSensor(Port.S4)


#saved_colours = open("savedColours.txt", "r")
preset_colours = {"Zone_1": Color.GREEN, "Zone_2": Color.BLUE,
                  "Zone_3": Color.RED, "Roundabout": Color.BROWN, "Warehouse_line": Color.YELLOW,
                  "Warehouse_background": Color.BLACK, "Background": Color.WHITE}

use_calibrator = False
going_to_target = False
set_colours = None
# Change to false to skip calibration mode and use .txt file if avalible

# Initialze the drivebase of the robot. Handles the motors (USE THIS)
TRUCK = DriveBase(left_motor=Right_drive, right_motor=Left_drive,
                  wheel_diameter=47, axle_track=128)

# Makes a start up sound
sound_start = EV3.speaker.beep()

# Speed:
DRIVING_INITAL = 20

# START


def startup():
    running = True
    while running:
        if Button.UP in EV3.buttons.pressed():
            if use_calibrator:
                # kör igång kalibrering
                EV3.speaker.say('Calibration start')
                EV3.speaker.play_file(SoundFile.READY)
                EV3.screen.print('Calibration start')
                print("Calibration started")
                wait(500)
                set_colours = Colour_Manager.Calibrate_Colours(
                    preset_colours, light_sensor)
        if Button.DOWN in EV3.buttons.pressed():
            set_colours = Colour_Manager.Get_File()
            print("colours read from file as : "+str(set_colours))
            use_calibrator = False
        elif Button.LEFT in EV3.buttons.pressed():
            # drive towards red warehouse
            EV3.speaker.say('Driving towards Red Warehouse')
            EV3.speaker.play_file(SoundFile.READY)
            EV3.screen.print('Driving towards Red Warehouse')
            print("Driving towards Red Warehouse")
            return [set_colours['Zone_1'], set_colours['Roundabout'], set_colours['Zone_2'], set_colours['Background']]
        elif Button.RIGHT in EV3.buttons.pressed():
            # drive towards blue warehouse
            EV3.speaker.say('Driving towards Blue Warehouse')
            EV3.speaker.play_file(SoundFile.READY)
            EV3.screen.print('Driving towards Blue Warehouse')
            print("Driving towards Blue Warehouse")
            return [set_colours['Zone_1'], set_colours['Roundabout'], set_colours['Zone_3'], set_colours['Background']]


def main():  # Main Class
    test_drive()


def test_drive():
    drive(startup())


def test_crane():
    Crane_motor.reset_angle(0)
    max_angle = crane_movement(Crane_motor, 1, 50)
    min_angle = crane_movement(Crane_motor, -1, 50)

    crane_pickup(Crane_motor, TRUCK, Front_button, -1000, max_angle, min_angle)
    crane_pickup(Crane_motor, TRUCK, Front_button,
                 max_angle/2, max_angle, min_angle)


def test_emergency_mode():
    pickup_on = True
    start_angle = crane_movement(Crane_motor, 1, 50)
    wait(5000)

    while pickup_on == True:
        wait(2000)
        print(Crane_motor.angle())
        pickup_on = emergency_mode(start_angle, Crane_motor)
    print("tappade :(")


def drive(list_rgb_colurs, background_color):
    """
    list_rgb_colurs - list, containing the colours to be on the lockout for

    Drives the robot towards the target zone, using a list of colours to determine it's path.
    Returns nothing.
    """

    drive_check = True
    pickupstatus = False

    list_of_colours = list_rgb_colurs
    index_of_colours = 0
    # Update to be a variable that is set by the startup function
    colour_one = background_color
    colour_two = list_of_colours[0]

    colour_one = rgb_to_hsv(colour_one[0], colour_one[1], colour_one[2])
    colour_two = rgb_to_hsv(colour_two[0], colour_two[1], colour_two[2])

    line_to_follow = colour_target(colour_one, colour_two)
    color_rgb = light_sensor.rgb()
    color_hsv = rgb_to_hsv(color_rgb[0], color_rgb[1], color_rgb[2])

    while drive_check is True:
        # Check the line it's following
        colour_two = list_of_colours[index_of_colours]

        # Check the line to follow
        line_to_follow = colour_target(colour_one, colour_two)
        color_rgb = light_sensor.rgb()
        color_hsv = rgb_to_hsv(color_rgb[0], color_rgb[1], color_rgb[2])

        # Check if the next colour is present
        # Add a threshold function that can accept slight deivance from the target

        if colour_deviation(light_sensor.rgb(), list_of_colours[index_of_colours + 1], 5) == True:
            index_of_colours += 1
            colour_two = list_of_colours[index_of_colours]

        if obstacle(300, "Driving", Ultrasonic_sensor) is True:
            EV3.speaker.say("There is an obstacle")
            EV3.speaker.play_file(SoundFile.OVERPOWER)
            TRUCK.stop()

        # print(sound_start)
        TRUCK.drive(DRIVING_INITAL, angle_to_colour(line_to_follow, color_hsv))
    return None


def warehouse_drive(colour_warehouse, drivebase, warehouse, max_angle, min_angle):
    """_summary_

    Args:
        colours (_type_): _description_
    """
    # Initialize variables
    ROBOT = drivebase
    distance_travled = 0
    continue_driving = True
    pickup_pallet = False

    # Check which way it's supposed to trun, depening on the warehouse
    if warehouse == "Red":
        turn_direction = 1
    elif warehouse == "Blue":
        turn_direction = -1

    # Check until you find a pallet
    while pickup_pallet is False:
        # Check if there is a pallet in front
        if obstacle(2500, "pallet_detection", Ultrasonic_sensor) is True:
            crane_pickup(Crane_motor, TRUCK, Front_button, -
                         1000, max_angle, min_angle)
        else:
            ROBOT.turn(90*turn_direction)
            # Drives until it finds the yellow line in the warehouse
            while continue_driving == True:
                # Might be a conflict if colour_warehouse is not RGB
                if colour_deviation(light_sensor.rgb(), colour_warehouse, 5) == True:
                    # Drive the same length it took to find the yellow line and turn
                    ROBOT.straight(distance_travled)
                    ROBOT.turn(-90*turn_direction)
                    continue_driving = False
                else:
                    # Drive small steps to find the yellow line
                    distance_travled += 10
                    ROBOT.straight(10)
    pass


def colour_deviation(colour_one, colour_two, deviation):
    """
    colour_one - list, containing the first colour in the RGB colour space
    colour_two - list, containing the second colour in the RGB colour space
    deviation - int, the amount of deviation allowed

    Returns if two colours are simillar enough, given a devitation
    """
    acceptable_deviation = True

    r_colour_one = colour_one[0]
    g_colour_one = colour_one[1]
    b_colour_one = colour_one[2]

    r_colour_two = colour_two[0]
    g_colour_two = colour_two[1]
    b_colour_two = colour_two[2]

    r_deviation = abs(r_colour_one - r_colour_two)
    g_deviation = abs(g_colour_one - g_colour_two)
    b_deviation = abs(b_colour_one - b_colour_two)

    if r_deviation < deviation:
        acceptable_deviation = False
    if g_deviation < deviation:
        acceptable_deviation = False
    if b_deviation < deviation:
        acceptable_deviation = False

    return acceptable_deviation


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
        EV3.speaker.play_file(SoundFile.OVERPOWER)


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

EV3.speaker.play_file(SoundFile.OVERPOWER)