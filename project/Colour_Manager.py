from sqlite3 import Row
from pybricks.hubs import EV3Brick as brick
from pybricks.parameters import Button
from numpy import save
from pybricks.tools import wait
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Color 

import ast

def Calibrate_Colours(colours,sensor):
    """
    Enter a calibraton mode where the user must press a button on the brick and set the colour value for each individual colour reading.

    Returns a dictionary of colours and saves to txt file
    """
    calibrated_colours = colours.copy()
    saved_colours =  open("savedColours.txt", "w")

    for label,colour in colours.items():
        #print("set colour for: "+label)
        #running = True
        #while running:
        #    if brick.buttons.pressed(): #if any button is pressed
        #        running = False
        #        brick.speaker.beep()
        selected_colour = colour#sensor.hsv()
        #^Uncomment for testing robot^
        saved_colours.write(label+":"+str(colour)+"\n")
    saved_colours.close()
    calibrated_colours
    return calibrated_colours
    
def Get_File():
    colours = {}
    saved_colours = open("savedColours.txt", "r")
    for colour_item in saved_colours.readlines():
        row = colour_item[:-1].split(":")
        print(row)
        colour = ast.literal_eval(row[1])
        colours[row[0]:colour]
    print(colours)
    saved_colours.close()

def get_colour(light_sensor):
    """
    Returns the colour of the ground the robot is looking at
    """
    return light_sensor.color()

def get_area(colours,light_sensor):
    threshold = 15
    current_colour = get_colour(light_sensor)
    for zone in colours:
        if current_colour == colours[zone]:
            #change: check if get_colour is within threshold of colours[zone], read colour will never be the same as saved colour 
            current_zone = zone
                # area has been switched
    return current_zone