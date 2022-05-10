from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import SoundFile
from Sensor_functions import obstacle
import math


def colour_target(color_1, color_2):

    line_to_follow = [((color_1[0] + color_2[0]) / 2),
                      ((color_1[1] + color_2[1]) / 2), ((color_1[2] + color_2[2]) / 2)]

    return line_to_follow


def angle_to_colour(line_to_follow, color_on_ground):
    angle = 0
    hue_diffrence = color_on_ground[0] - line_to_follow[0]
    saturation_diffrence = color_on_ground[1] - line_to_follow[1]
    value_diffrence = color_on_ground[2] - line_to_follow[2]
    # Get a value that is between -1 and 1
    sum_of_diffrence = (
        hue_diffrence + saturation_diffrence + value_diffrence) / 300

    angle = 90 * sum_of_diffrence

    return angle


def angle_to_speed(speed, angle, factor):
    """
    speed - int, the speed to be used
    angle - int, the angle to be used
    Returns the speed to be used
    """

    angle = abs(angle)

    try:
        speed = factor*speed * 1/angle
    except:
        speed = speed

    return speed


def turn_around(Drivebase, Ultrasonic_sensor):
    TRUCK = Drivebase
    while obstacle(300, "Driving", Ultrasonic_sensor) is True:
        wait(1000)
    TRUCK.straight(140)
    TRUCK.turn(-90)

    while obstacle(300, "Driving", Ultrasonic_sensor) is True:
        wait(1000)
    TRUCK.straight(140)
    TRUCK.turn(-90)
