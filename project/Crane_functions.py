#!/usr/bin/env pybricks-micropython
from pybricks.parameters import Stop
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from Sensor_functions import button_pressed
from Drive_functions import angle_to_colour, colour_target, angle_to_speed, turn_around
from Beep_Pack import *
import math

EV3 = EV3Brick()
crane_motor = Motor(Port.A, gears=[12, 36])
Front_button = TouchSensor(Port.S1)
light_sensor = ColorSensor(Port.S3)
Ultrasonic_sensor = UltrasonicSensor(Port.S4)

resting_angle = crane_motor.angle()

elevated_offset = 0.5
#all values in cm
pallet_height = 11.0
pallet_length = 6.0

pivot_height = 3.3
fork_length = 14.0
#^do not set this to zero

def set_crane_rotation(height, speed):
    catheus = (height - pivot_height) + elevated_offset
    if catheus > 0:
        target_angle =  math.degrees(math.asin(catheus/fork_length))
    else:
        target_angle = 0
    print(target_angle)
    crane_motor.run_target(speed,target_angle,then=Stop.HOLD,wait = True)
    
def pick_up_pallet(speed,timeout,truck,height= pallet_height):
    """timeout - maximum amount of iterations to look for button press before aborting"""
    set_crane_rotation(height, speed)
    while Front_button.pressed() == False or timeout > 0:
        truck.drive(10,0)
        timeout -= 1
        wait(10)
    super_beep(1000)
    
    truck.stop()
    set_crane_rotation(height+5,speed)
    crane_motor.hold()
    truck.straight(-pallet_length*10)
    set_crane_rotation(0,speed)
    turn_around(truck,Ultrasonic_sensor)

def crane_movement(direction, speed):  # Function for moving the crane up
    """
    Crane_port - Class contatning the port, containing the port of the crane
    direction, a value between -1 and 1, indicating the direction of the movement
    speed, a value between 0 and 100, indicating the speed of the movement
    Returns an angle of the crane at it's maximum angle
    """
    crane_motor.stop()

    speed_of_crane = speed * direction
    return crane_motor.run_until_stalled(speed_of_crane, then=Stop.BRAKE, duty_limit=50)


def crane_hold():  # Function for moving the crane up
    """
    Crane_port - Class contatning the port, containing the port of the crane

    Returns an angle of the crane at it's maximum angle
    """
    # To prevent problems with the crane holding
    speed_of_crane = 50
    return crane_motor.run_until_stalled(speed_of_crane, Stop.HOLD, duty_limit=90)

# Function for moving the crane up


def crane_pickup(DriveBase, angle_of_crane, background, line_colour):
    """
    Crane_port - Class containing the port, containing the port of the crane
    DriveBase - Class that handles the drving of the robot
    Front_button - Class that handles the button on the front of the robot
    angle_of_crane - Int, containing the angle the crane should be
    max_angle - int, containing the maximum angle the crane can be
    min_angle - int, containing the minimum angle the crane can be

    Returns None
    """
    # Initializing the variables
    speed_of_crane = 50
    ROBOT = DriveBase
    DRIVING_INITAL = 20
    pickupstatus = False
    pallet_found = button_pressed(Front_button)

    # Raise the crane to the angle of the pallet
    crane_motor.run_target(speed=speed_of_crane,
                           target_angle=angle_of_crane, wait=False)

    # Drive forward untill the button is pressed
    while pallet_found is False:
        pallet_found = button_pressed(Front_button)
        # Get the RGB of the background
        colour_rgb = light_sensor.rgb()
        # Get the line to follow
        line_to_follow = colour_target(line_colour, background)
        # get the new angle
        angle = angle_to_colour(line_to_follow, colour_rgb)
        # get the speed
        speed = angle_to_speed(DRIVING_INITAL, angle, 1)

        # Drive
        ROBOT.drive(speed, angle)
    raise_incremental(angle_of_crane)
    pickupstatus = True
    EV3.speaker.beep()
    return pickupstatus


def raise_incremental(angle_at_start):
    """Raise the crane 10 degrees
    """
    crane_motor.stop()
    angle = angle_at_start + 10
    current_angle = angle_at_start
    while current_angle < angle:
        crane_motor.track_target(angle)
        current_angle = crane_motor.angle()

    crane_motor.hold()

    return None
