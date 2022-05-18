from pybricks.tools import wait
from pybricks.hubs import EV3Brick
import math
import time
import random

brick = EV3Brick()
def Siren(beep_frequency, sine_frequency):
    """call this inside a while loop for desired effect"""
    threshold = 0.8
    sine_wave = abs(math.sin(time.time()*sine_frequency))
    if sine_wave >= threshold:
        brick.speaker.beep(beep_frequency)


def super_beep(frequency):
    """Super beep, makes a speical beep sound effect"""
    for i in range(5):
        brick.speaker.beep(frequency*i)
        wait(50)

def seizure(amount,min,max):
    for i in range(amount):
        brick.speaker.beep(random.randrange(min,max))
        wait(2)