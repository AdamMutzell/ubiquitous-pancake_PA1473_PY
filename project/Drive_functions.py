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


def curve(factor, x, m):
    return factor * x**2+m


def interpolate_motor_movement(initial_speed, target_angle, dampener_amount, motor):
    moving = True
    iteration = 0
    while moving:
        current_drive_speed = initial_speed * \
            curve(-dampener_amount, iteration, 1)
        if motor.angle != target_angle:
            motor.run()
        iteration += 1
        wait(2)


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
    # TRUCK.straight(140)
    TRUCK.turn(-90)

    while obstacle(300, "Driving", Ultrasonic_sensor) is True:
        wait(1000)
    # TRUCK.straight(140)
    TRUCK.turn(-90)


def change_route(button_input, list_of_colors, current_color, other_route):
    if button_input == 'LEFT':
        # If the last Blue-value of the last color is greater than the other route's Blue-value
        # Then it should stay on route.
        if list_of_colors[2][2] < other_route[2]:
            EV3Brick().speaker.say("Already driving towards Red Warehouse")
        elif current_color[0] < other_route[0] and current_color == list_of_colors[2]:
            EV3Brick().speaker.say("Turning around to Red Warehouse")
            temp_variable = list_of_colors[2]
            list_of_colors[2] = other_route
            other_route = temp_variable
        else:
            EV3Brick().speaker.say("Change Route to Red Warehouse")
            temp_variable = list_of_colors[2]
            list_of_colors[2] = other_route
            other_route = temp_variable
    elif button_input == 'RIGHT':
        if list_of_colors[2][0] < other_route[0]:
            EV3Brick().speaker.say("Already driving towards Blue Warehouse")
        elif current_color[2] < other_route[2] and current_color == list_of_colors[2]:
            EV3Brick().speaker.say("Turning around to Blue Warehouse")
            temp_variable = list_of_colors[2]
            list_of_colors[2] = other_route
            other_route = temp_variable
        else:
            EV3Brick().speaker.say("Change Route to Blue Warehouse")
            temp_variable = list_of_colors[2]
            list_of_colors[2] = other_route
            other_route = temp_variable
    return list_of_colors, other_route
