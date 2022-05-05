#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import SoundFile
import time

EV3 = EV3Brick()

start_time = 10


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
        if button_pressed is False:
            if start_time - time.time() > 5:
                emergency_mode()
                return False
        elif button_pressed is True:
            start_time = time.time()
            return True
    return False
