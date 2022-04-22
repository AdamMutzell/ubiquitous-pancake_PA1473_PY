from pybricks.hubs import EV3Brick as brick
from pybricks.parameters import Button
from numpy import save
from pybricks.tools import wait
from pybricks.ev3devices import ColorSensor

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
        calibrated_colours[label] = selected_colour
        saved_colours.write(str(selected_colour)+"\n")
    saved_colours.close()
    print(calibrated_colours)
    return calibrated_colours
    
def Get_File():
    saved_colours = open("savedColours.txt", "r")
    colours = saved_colours.read().split("\n")
    print(colours)
    saved_colours.close()

def get_colour(light_sensor):
    """
    Returns the colour of the ground the robot is looking at
    """
    return light_sensor.color()


def get_area(colours, current_area):
    threshold = 15
    current_colour = get_colour()
    for colour in colours:
        if get_colour() == colour:
            if colour != current_area:
                current_area = colour
                # area has been switched
    return current_area