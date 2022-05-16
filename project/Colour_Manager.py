from pybricks.hubs import EV3Brick as brick
from pybricks.tools import wait
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port

light_sensor = ColorSensor(Port.S3)

def set_colour_history(colours_to_compare,current_colour_history):
    for colour in colours_to_compare.keys():
        print(colours_to_compare[colour])
        if colour_deviation(light_sensor.rgb(),colours_to_compare[colour],15):
            if colours_to_compare[colour] not in current_colour_history:
                current_colour_history.append(colours_to_compare[colour])
                current_colour_history.pop(0)
                print(current_colour_history.reverse())
    return current_colour_history

def Calibrate_Colours(colour_labels, EV3):
    """
    Enter a calibraton mode where the user must press a button on the brick and set the colour value for each individual colour reading.

    Returns a dictionary of colour_labels and saves to txt file
    """
    selected_colour = ()
    saved_colours = open("savedColours.txt", "w")
    colour_dict = {}
    for label in colour_labels.keys():
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
        print('saved')
        for colour_item in saved_colours.readlines():
            print('read')
            row = colour_item[:-1].split(":")
            label = row[0]
            colour_string = row[1][1:-1].split(",")
            print(colour_string)
            colour = []
            for value in colour_string:
                try:
                    colour.append(int(value))
                except:
                    print(value)
            colour = tuple(colour)
            colours[label] = colour
        saved_colours.close()
        return colours
    except FileNotFoundError:
        print("no .txt file found! returning None")
        saved_colours = open("savedColours.txt", "x")
        return None

def colour_deviation(colour_one, colour_two, deviation):
    """
    colour_one - list, containing the first colour in the RGB colour space
    colour_two - list, containing the second colour in the RGB colour space
    deviation - int, the amount of deviation allowed

    Returns if two colours are simillar enough, given a devitation
    """
    # Check if the colours are simillar enough
    acceptable_deviation = False

    r_colour_one = colour_one[0]
    g_colour_one = colour_one[1]
    b_colour_one = colour_one[2]

    r_colour_two = colour_two[0]
    g_colour_two = colour_two[1]
    b_colour_two = colour_two[2]

    r_deviation = abs(r_colour_one - r_colour_two)
    g_deviation = abs(g_colour_one - g_colour_two)
    b_deviation = abs(b_colour_one - b_colour_two)

    if r_deviation > deviation:
        acceptable_deviation = False
    elif g_deviation > deviation:
        acceptable_deviation = False
    elif b_deviation > deviation:
        acceptable_deviation = False
    else:
        acceptable_deviation = True

    return acceptable_deviation


def get_colour(light_sensor):
    """
    Returns the colour of the ground the robot is looking at
    """
    return light_sensor.rgb()
