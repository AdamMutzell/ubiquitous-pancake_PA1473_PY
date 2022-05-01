#!/usr/bin/env pybricks-micropython
import sys
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
                  "Zone_3": Color.RED, "Roundabout": Color.BROWN, "Warehouse": Color.YELLOW}

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
                #EV3Brick.screen.print('Calibration start')
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
            #EV3Brick.screen.print('Driving towards Red Warehouse')
            print("Driving towards Red Warehouse")
            return [set_colours['Zone_1'], set_colours['Roundabout'], set_colours['Zone_2']]
        elif Button.RIGHT in EV3.buttons.pressed():
            # drive towards blue warehouse
            #EV3Brick.screen.print('Driving towards Blue Warehouse')
            print("Driving towards Blue Warehouse")
            return [set_colours['Zone_1'], set_colours['Roundabout'], set_colours['Zone_3']]


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


def drive(list_rgb_colurs):
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
    colour_one = [43, 60, 86]
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
        if light_sensor.rgb() == list_of_colours[index_of_colours + 1]:
            index_of_colours += 1
            colour_two = list_of_colours[index_of_colours]

        if obstacle(300, "Driving", Ultrasonic_sensor) is True:
            TRUCK.stop()

        # print(sound_start)
        TRUCK.drive(DRIVING_INITAL, angle_to_colour(line_to_follow, color_hsv))
    return None


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
