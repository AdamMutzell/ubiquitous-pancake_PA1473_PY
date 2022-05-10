from pybricks.hubs import EV3Brick as brick
from pybricks.tools import wait
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port

light_sensor = ColorSensor(Port.S3)
colour_history = [0,0,0]

def Calibrate_Colours(colours, EV3):
    """
    Enter a calibraton mode where the user must press a button on the brick and set the colour value for each individual colour reading.

    Returns a dictionary of colours and saves to txt file
    """
    selected_colour = ()
    saved_colours = open("savedColours.txt", "w")
    colour_dict = {}
    for label, colour in colours.items():

        print("set colour for: "+label)
        EV3.screen.print(label)
        # add robot voice command here?
        wait(500)
        running = True
        while running:
            if brick.buttons.pressed():  # if any button is pressed

                selected_colour = get_colour(light_sensor)
                wait(500)
                running = False
        print(selected_colour)
        saved_colours.write((label+":"+str(selected_colour)+"\n"))
        colour_dict[label] = selected_colour
    saved_colours.close()
    EV3.speaker.say('Calibration done')
    return colour_dict


def Get_File():
    """Get savedColours.txt and return content as a dictionary"""
    colours = {}
    try:
        saved_colours = open("savedColours.txt", "r")

        for colour_item in saved_colours.readlines():
            row = colour_item[:-1].split(":")
            label = row[0]
            colour_string = row[1][1:-2].split(",")
            colour = []
            for value in colour_string:
                colour.append(int(value))
            colour = tuple(colour)
            colours[label] = colour
        saved_colours.close()
        return colours
    except:
        print("no .txt file found! returning None")
        return None

def get_colour(light_sensor):
    """
    Returns the colour of the ground the robot is looking at
    """
    return light_sensor.rgb()