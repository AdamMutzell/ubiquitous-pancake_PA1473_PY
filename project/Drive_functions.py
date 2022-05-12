from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import SoundFile
from Sensor_functions import obstacle
import math

EV3 = EV3Brick()


def colour_target(color_1, color_2):
    """
    Takes two colours and get's the colour in between those values

    color_1 - list: Containing RGB values
    color_2 - list: Containing RGB values

    Returs a mix of the two colours
    """

    line_to_follow = [((color_1[0] + color_2[0]) / 2),
                      ((color_1[1] + color_2[1]) / 2), ((color_1[2] + color_2[2]) / 2)]

    return line_to_follow


def angle_to_colour(line_to_follow, color_on_ground):
    """Takes in two colours and returns the estimated angle between them

    line_to_follow - list: Containing RGB values
    color_on_ground - list: Containing RGB values

    Returns the estimated angle between the two colours
    """
    angle = 0
    hue_diffrence = color_on_ground[0] - line_to_follow[0]
    saturation_diffrence = color_on_ground[1] - line_to_follow[1]
    value_diffrence = color_on_ground[2] - line_to_follow[2]
    # Get a value that is between -1 and 1
    sum_of_diffrence = (
        hue_diffrence + saturation_diffrence + value_diffrence) / 300

    angle = 90 * sum_of_diffrence

    return -angle


def angle_to_speed(speed, angle, factor):
    """
    speed - int, the speed to be used
    angle - int, the angle to be used
    Returns the speed to be used
    """

    angle = abs(angle)

    try:
        speed = factor*speed
    except:
        speed = speed

    return speed


def turn_around(Drivebase, Ultrasonic_sensor):
    """Turns the robot around

    Returns none"""

    TRUCK = Drivebase
    while obstacle(300, "Driving", Ultrasonic_sensor) is True:
        wait(1000)
    TRUCK.straight(140)
    TRUCK.turn(-90)

    while obstacle(300, "Driving", Ultrasonic_sensor) is True:
        wait(1000)
    TRUCK.straight(140)
    TRUCK.turn(-90)


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


def change_route(button_input, list_of_colors, current_color, other_route):
    if button_input == 'LEFT':
        # If the last Blue-value of the last color is greater than the other route's Blue-value
        # Then it should stay on route.
        if list_of_colors[2][2] < other_route[2]:
            EV3Brick().speaker.say("Already driving towards Red Warehouse")
        elif current_color[0] < other_route[0] and current_color == list_of_colors[2]:
            EV3Brick().speaker.say("Turning around to Red Warehouse")
        else:
            EV3Brick().speaker.say("Change Route to Red Warehouse")
            list_of_colors[2] = other_route
            return list_of_colors
    elif button_input == 'RIGHT':
        if list_of_colors[2][0] > other_route[0]:
            EV3Brick().speaker.say("Already driving towards Blue Warehouse")
        elif current_color[2] > other_route[2] and current_color == list_of_colors[2]:
            EV3Brick().speaker.say("Turning around to Blue Warehouse")
        else:
            EV3Brick().speaker.say("Change Route to Blue Warehouse")
            list_of_colors[2] = other_route
    return list_of_colors
