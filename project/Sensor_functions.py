#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
import time

EV3 = EV3Brick()


def button_pressed(Front_button):  # Function for detecting button press
    """
    Front_button, class handling the front button of the robot

    Returns true if the button is pressed, false otherwise
    """

    if Front_button.pressed():
        return True
    else:
        return False

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