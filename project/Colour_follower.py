from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import SoundFile
import math


# Python3 program change RGB Color
# Model to HSV Color Model

def rgb_to_hsv(r, g, b):
    r, g, b = r / 100, g / 100, b / 100
    # h, s, v = hue, saturation, value
    cmax = max(r, g, b)    # maximum of r, g, b
    cmin = min(r, g, b)    # minimum of r, g, b
    diff = cmax-cmin       # diff of cmax and cmin.

    # if cmax and cmax are equal then h = 0
    if cmax == cmin:
        h = 0

    # if cmax equal r then compute h
    elif cmax == r:
        h = (60 * ((g - b) / diff) + 360) % 360

    # if cmax equal g then compute h
    elif cmax == g:
        h = (60 * ((b - r) / diff) + 120) % 360

    # if cmax equal b then compute h
    elif cmax == b:
        h = (60 * ((r - g) / diff) + 240) % 360

    # if cmax equal zero
    if cmax == 0:
        s = 0
    else:
        s = (diff / cmax) * 100

    # compute v
    v = cmax * 100
    return h, s, v


def colour_target(color_1, color_2):

    line_to_follow = Color((color_1.h + color_2.h) / 2,
                           (color_1.s + color_2.s) / 2, (color_1.v + color_2.v) / 2)

    return line_to_follow


def angle_to_colour(line_to_follow, color_on_ground):
    angle = 0
    hue_diffrence = color_on_ground.h - line_to_follow.h
    saturation_diffrence = color_on_ground.s - line_to_follow.s
    value_diffrence = color_on_ground.v - line_to_follow.v
    # Get a value that is between -1 and 1
    sum_of_diffrence = (
        hue_diffrence + saturation_diffrence + value_diffrence) / 455

    print(sum_of_diffrence)

    angle = 90 * sum_of_diffrence

    return angle
