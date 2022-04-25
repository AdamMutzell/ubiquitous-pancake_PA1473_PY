import sys
from tkinter import Button
from turtle import color, left, right
import __init__
import Colour_Manager

import math,time

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor, Button
from pybricks.parameters import Port, Color, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import SoundFile

if EV3Brick.Button().left:
    #drive towards red warehouse
    print("Driving towards Red Warehouse")
elif EV3Brick.Button().right:
    #drive towards blue warehouse
    print("Driving towards Blue Warehouse")


#START
Button().wait_for_bump(['up', 'left', 'right'])
if Button().up:
    #kör igång calibrering
    EV3Brick.screen.print('Calibration start')
elif Button().left:
    #drive towards red warehouse
    EV3Brick.screen.print('Driving towards Red Warehouse')
elif Button().right:
    #drive towards blue warehouse
    EV3Brick.screen.print('Driving towards Blue Warehouse')
