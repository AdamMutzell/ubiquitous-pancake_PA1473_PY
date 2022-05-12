#!/usr/bin/env pybricks-micropython
import sys
import __init__
from Drive_functions import angle_to_colour, colour_target, angle_to_speed, colour_deviation, change_route
from Crane_functions import crane_movement, crane_pickup
from Sensor_functions import button_pressed, obstacle
from Colour_Manager import Calibrate_Colours, Get_File
import math
import time
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color, Direction, Button, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import SoundFile

from Drive_functions import turn_around

# Initialise the EV3
EV3 = EV3Brick()

# Initialzie the components of the robot
Crane_motor = Motor(Port.A, gears=[12, 36])
Right_drive = Motor(
    Port.B, positive_direction=Direction.COUNTERCLOCKWISE, gears=[12, 20])
Left_drive = Motor(
    Port.C, positive_direction=Direction.COUNTERCLOCKWISE, gears=[12, 20])

Front_button = TouchSensor(Port.S1)
light_sensor = ColorSensor(Port.S3)
Ultrasonic_sensor = UltrasonicSensor(Port.S4)


# Initizles colours and directions for the robot
# direction = ""
# colour_history = [(0,0,0),(0,0,0),(0,0,0)]
preset_colours = {"Zone_1": Color.GREEN, "Zone_2": Color.RED,
                   "Zone_3": Color.BLUE, "Roundabout": Color.BROWN, "Warehouse_line": Color.YELLOW,
                   "Warehouse_start": Color.BLACK, "Warehouse_blue": Color.BLUE, "Warehouse_red": Color.RED, "Background": Color.WHITE}

# set_colours = preset_colours
# Initizles start up statments
# Change to false to skip calibration mode and use .txt file if avalible
pickupstatus = False
start_time = None
DRIVING_INITAL = 1

# Initialze the drivebase of the robot. Handles the motors (USE THIS)
TRUCK = DriveBase(left_motor=Right_drive, right_motor=Left_drive,
                  wheel_diameter=47, axle_track=128)

# Makes a start up sound, to signify everything went well
sound_start = EV3.speaker.beep()


# START
def startup():
    """
    A function that handles the startup procedures of the robot, allowing a user to...
    ...choose to calibrate, or use a previously saved file.
    ...drive twoards a specfic place in the warehouse.

    Returns a list of colours, the background colour, the warehouse colour, and the warehouse line colour
    """
    running = True
    while running:
        if Button.UP in EV3.buttons.pressed():
            # kör igång kalibrering
            EV3.speaker.say('Calibration start')
            EV3.speaker.play_file(SoundFile.READY)
            EV3.screen.print('Calibration start')
            print("Calibration started")
            wait(500)
            set_colours = Calibrate_Colours(
                preset_colours, EV3)
        if Button.DOWN in EV3.buttons.pressed():
            EV3.speaker.say('Using last calibration')
            set_colours = Get_File()
            print("colours read from file as : "+str(set_colours))
            wait(500)
        elif Button.LEFT in EV3.buttons.pressed():
            # drive towards red warehouse
            EV3.speaker.say('Driving towards Red Warehouse')
            EV3.speaker.play_file(SoundFile.READY)
            EV3.screen.print('Driving towards Red Warehouse')
            print("Driving towards Red Warehouse")
            return ([set_colours['Zone_1'], set_colours['Roundabout'], set_colours['Zone_2'],
                    set_colours['Warehouse_start']], set_colours['Background'],
                    set_colours["Warehouse_red"], set_colours["Warehouse_line"], set_colours['Zone_3'])
        elif Button.RIGHT in EV3.buttons.pressed():
            # drive towards blue warehouse
            EV3.speaker.say('Driving towards Blue Warehouse')
            EV3.speaker.play_file(SoundFile.READY)
            EV3.screen.print('Driving towards Blue Warehouse')
            print("Driving towards Blue Warehouse")
            return ([set_colours['Zone_1'], set_colours['Roundabout'], set_colours['Zone_3'],
                    set_colours['Warehouse_start']], set_colours['Background'],
                    set_colours["Warehouse_blue"], set_colours["Warehouse_line"], set_colours['Zone_2'])


def main():  # Main Class
    test_drive()


def test_drive():
    list_of_colours, colour_background, warehouse_colour, warehouse_line, alt_route = startup()
    print(list_of_colours, colour_background, warehouse_colour, warehouse_line)
    drive(list_of_colours, colour_background,
          warehouse_colour, warehouse_line, alt_route)


def test_warehouse():
    warehouse_line = (48, 37, 6)
    warehouse_blue = (6, 6, 8)
    warehouse_colour = warehouse_blue
    warehouse_drive(light_sensor, TRUCK, warehouse_colour, warehouse_line)


def drive(list_rgb_colurs, background_color, warehouse_colour, warehouse_line, alt_route):
    """
    list_rgb_colurs - list, containing the colours to be on the lockout for
    background_color - list, the colour to be used as background
    Drives the robot towards the target zone, using a list of colours to determine it's path.
    Returns nothing.
    """
    angle = 0
    speed = 0

    list_of_colours = list_rgb_colurs
    index_of_colours = 0
    on_line = False
    drive_check = True
    # Update to be a variable that is set by the startup function
    colour_one = background_color
    colour_two = list_of_colours[0]

    line_to_follow = colour_target(colour_one, colour_two)
    color_rgb = light_sensor.rgb()

    # Print the hsv it's on
    EV3.screen.print("Following a line")

    while drive_check is True:
        # Check the line it's following
        colour_two = list_of_colours[index_of_colours]

        # Check the line to follow
        line_to_follow = colour_target(colour_one, colour_two)
        color_rgb = light_sensor.rgb()

        # Check if the next colour is present
        if colour_deviation(color_rgb, list_of_colours[index_of_colours + 1], 4) == True:
            index_of_colours += 1
            colour_two = list_of_colours[index_of_colours]
            # Say that it has changed colours
            EV3.screen.print('New colour found')
            TRUCK.drive(0, -30)
            wait(800)

        # Check if we want to change route
        if Button.LEFT in EV3.buttons.pressed():
            list_of_colours, alt_route = change_route('LEFT',list_of_colours,colour_two,alt_route)
        elif Button.RIGHT in EV3.buttons.pressed():
            list_of_colours, alt_route = change_route('RIGHT',list_of_colours,colour_two,alt_route)
        # Emergency mode

        # if pickupstatus is True and detect_item_fail(Front_button, pickupstatus) is False:
        #    pickupstatus = False
        #    EV3.screen.print('Driving back')
        #    TRUCK.turn(180)
        #    reversed_list = list_rgb_colurs[0:index_of_colours]
        #    reversed_list = reversed_list[::-1]
        #    drive(reversed_list, background_color,
        #          warehouse_colour, warehouse_line, EV3)
        #           """"

        if obstacle(200, "Driving", Ultrasonic_sensor) is True:
            TRUCK.stop()
            EV3.speaker.say("There is an obstacle")
            EV3.speaker.play_file(SoundFile.OVERPOWER)
            TRUCK.stop()

        # Check if we are at the end of the list
        if index_of_colours == len(list_of_colours) - 1:
            drive_check = False
        # get the new angle
        angle = angle_to_colour(line_to_follow, color_rgb)
        # get the speed
        speed = angle_to_speed(DRIVING_INITAL, angle, 3)
        # drive the robot
        if colour_deviation(color_rgb, colour_two, 4) is True:
            on_line = True
            TRUCK.drive(0, 7)
            wait(400)
        while on_line is True:

            TRUCK.drive(-50, 20)

            color_rgb = light_sensor.rgb()
            on_line = colour_deviation(color_rgb, colour_two, 4)

        TRUCK.drive(speed, angle)
    return None


def warehouse_drive(light_sensor, drivebase, warehouse, line_warehouse):
    """_summary_

    Args:
        colours (_type_): _description_
    """
    # Initialize variables
    ROBOT = drivebase
    distance_travled = 0
    drive_speed = 50
    turn_factor = 10
    continue_driving = True
    straight_on_line = False
    pickup_pallet = False
    enter_pickup = False

    # Check which way it's supposed to turn, depening on the warehouse
    # Red warehouse
    if warehouse[0] >= warehouse[2]:
        turn_direction = -1
        turn_factor = 45
        # Say which warehouse you are in
        EV3.speaker.say('In the Red Warehouse')
    # Blue warehouse
    elif warehouse[2] >= warehouse[0]:
        turn_direction = 1
        turn_factor = 90
        # Say which ware you are in
        EV3.speaker.say('In the Blue Warehouse')

    # make a turn so that it's facing the middle of the warehouse
    ROBOT.turn(turn_direction * turn_factor)

    # Drive untill you find the yellow line
    while continue_driving is True:
        # Drive slightly towards the right untill it find white or the line
        # Check if the robot is on the line
        if colour_deviation(light_sensor.rgb(), line_warehouse, 4) is True:
            # If it is, stop the robot
            ROBOT.stop()
            # Say that it has found the line
            EV3.screen.print('Line found')
            # Wait for the robot to stop
            continue_driving = False
        ROBOT.drive(drive_speed, 0)
    # Make sure it's straight on the yellow line
    ROBOT.reset()
    while straight_on_line is False:

        # Check if the robot ses an object infront of it
        if obstacle(50, "Driving", Ultrasonic_sensor) is True and ROBOT.distance() > 100:
            # enter crane pickup
            straight_on_line = True
        # If not, continue driving on the line until you reach white
        line_to_follow = colour_target(warehouse, line_warehouse)
        angle = angle_to_colour(line_to_follow, light_sensor.rgb())
        drive_speed = angle_to_speed(DRIVING_INITAL, angle, 3)
        ROBOT.drive(drive_speed, angle)

    crane_pickup(ROBOT, 0, warehouse, line_warehouse)
    pass

    # Check until you find a pallet
    while pickup_pallet is False:
        # Check if there is a pallet in front
        if obstacle(1000, "pallet_detection", Ultrasonic_sensor) is True:
            enter_pickup = True
            # Go to the next area
        ROBOT.turn(90*turn_direction)
        # Drives until it finds the yellow line in the warehouse
        while continue_driving == True:
            # Might be a conflict if colour_warehouse is not RGB
            if colour_deviation(light_sensor.rgb(), colour_warehouse, 5) == True:
                # Drive the same length it took to find the yellow line and turn
                ROBOT.turn(-90*turn_direction)
                if enter_pickup is True:
                    crane_pickup(TRUCK, 1000, max_angle, min_angle)
                else:
                    ROBOT.straight(distance_travled)
                    continue_driving = False
            else:
                # Drive small steps to find the yellow line
                distance_travled += 10
                ROBOT.straight(10)
        continue_driving = True

    pass


def zig_zag_angle(drivebase, colour_on_ground, turn_angle):
    ROBOT = drivebase
    # Turn the robot
    ROBOT.turn(turn_angle)
    # Wait
    ROBOT.turn(-2*turn_angle)


def Siren(beep_frequency, sine_frequency):
    """call this inside a while loop for desired effect"""
    threshold = 0.8
    sine_wave = abs(math.sin(time.time()*sine_frequency))
    if sine_wave >= threshold:
        EV3.speaker.play_file(SoundFile.OVERPOWER)


def Super_Beep():
    for i in range(5):
        EV3.speaker.beep(500*i)
        wait(50)


# def exit_zone(initial_zone):
#     set_colour_history()
#     #direction = get_direction(colour_history)

# def set_colour_history():
#     for colour in set_colours.keys():
#         if colour_deviation(light_sensor.rgb(),set_colours[colour],15):
#             colour_history.append(colour)
#             colour_history.pop(0)

# def get_direction_towards(colour_history):
#     for colour in colour_history.reverse():
#         for label in set_colours.keys:
#             if "Warehouse" in label:
#                 if (colour == set_colours[label]):
#                     direction = "Warehouse"
#                     break
#         if (colour == set_colours["Warehouse"]):
#             direction = "Roundabout"
#             break
#     return direction

def detect_item_fail(stat):
    """
    pickupstatus - boolean, True if the truck is currently picking up an item
    button, a class handling the front button of the robot

    Returns True if the pickup has failed, False otherwise
    """
    global start_time
    if stat == True:
        if button_pressed(Front_button) == False:
            # For the first iteration, start the timer.
            if start_time == None:
                start_time = time.time()
            # If an object haven't been on the crane for 5 seconds, make a sound
            if time.time() - start_time > 5:
                Super_Beep()
                start_time = None
            else:
                return True
        # If object is on the crane, reset the timer
        elif button_pressed(Front_button) == True:
            start_time = time.time()
            return True
    return False


if __name__ == '__main__':  # Keep this!
    sys.exit(main())

EV3.speaker.play_file(SoundFile.OVERPOWER)
