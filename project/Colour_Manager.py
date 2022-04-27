from sqlite3 import Row
from pybricks.hubs import EV3Brick as brick
from pybricks.parameters import Button
from numpy import save
from pybricks.tools import wait
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port 

light_sensor = ColorSensor(Port.S3)
import ast

def Calibrate_Colours(colours,sensor):
    """
    Enter a calibraton mode where the user must press a button on the brick and set the colour value for each individual colour reading.

    Returns a dictionary of colours and saves to txt file
    """
    calibrated_colours = colours.copy()
    saved_colours =  open("savedColours.txt", "w")

    for label,colour in colours.items():
        print("set colour for: "+label)
        running = True
        while running:
            if brick.buttons.pressed(): #if any button is pressed
                brick.speaker.beep()
                selected_colour = get_colour(light_sensor.rgb())
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
        label = row[0]
        colour_string = row[1][1:-1].split(",")
        print(row)
        colour = []
        for value in colour_string:
            print(value)
            colour.append(int(value))
        colour = tuple(colour)
        print(colour)
        colours[label] = colour
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

colours = {"Zone_1": (1,2,3), "Zone_2": (2,3,4),
           "Zone_3": (5,6,7), "Roundabout": (8,9,10), "Warehouse": (11,12,13)}
Calibrate_Colours(colours,3)
Get_File()