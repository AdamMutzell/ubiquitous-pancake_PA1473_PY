import sys
from turtle import color, left, right
import __init__
import Colour_Manager

import math
import time

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import SoundFile

btn = EV3Brick.Button()


# START
btn.wait_for_bump(['up', 'left', 'right'], 2000)
if btn.up:
    # kör igång calibrering
    EV3Brick.screen.print('Calibration start')
elif btn.left:
    # drive towards red warehouse
    EV3Brick.screen.print('Driving towards Red Warehouse')
elif btn.right:
    # drive towards blue warehouse
    EV3Brick.screen.print('Driving towards Blue Warehouse')
