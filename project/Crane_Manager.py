#!/usr/bin/env pybricks-micropython
from pybricks.parameters import Stop
from Sensor_Manager import button_pressed


def crane_movement(Crane_motor, direction, speed):  # Function for moving the crane up
    """
    Crane_port - Class contatning the port, containing the port of the crane
    direction, a value between -1 and 1, indicating the direction of the movement
    speed, a value between 0 and 100, indicating the speed of the movement
    Returns an angle of the crane at it's maximum angle
    """
    Crane_motor.stop()

    speed_of_crane = speed * direction
    return Crane_motor.run_until_stalled(speed_of_crane, then=Stop.BRAKE, duty_limit=50)


def crane_hold(Crane_motor):  # Function for moving the crane up
    """
    Crane_port - Class contatning the port, containing the port of the crane

    Returns an angle of the crane at it's maximum angle
    """
    # To prevent problems with the crane holding
    Crane_motor.stop()

    speed_of_crane = 50
    return Crane_motor.run_until_stalled(speed_of_crane, Stop.HOLD, duty_limit=90)

# Function for moving the crane up


def crane_pickup(Crane_motor, DriveBase, Front_button, angle_of_crane, max_angle, min_angle):
    """
    Crane_port - Class containing the port, containing the port of the crane
    DriveBase - Class that handles the drving of the robot
    Front_button - Class that handles the button on the front of the robot
    angle_of_crane - Int, containing the angle the crane should be
    max_angle - int, containing the maximum angle the crane can be
    min_angle - int, containing the minimum angle the crane can be

    Returns None
    """
    # To do, check out stop function on target and stall limits

    # Initializing the variables
    speed_of_crane = 50
    raise_angle = 50
    distance_traveled = 0
    ROBOT = DriveBase

    # Seetings for the crane motor
    Crane_motor.control.stall_tolerances(stall_limit=90, stall_time_limit=5000)

    # Makes sure that the angle of the crane is valid
    if angle_of_crane < min_angle:
        angle_of_crane = min_angle

    # Raise the crane to the angle of the pallet
    Crane_motor.run_target(speed_of_crane, angle_of_crane, )

    # Drive forward for 100mm
    while button_pressed(Front_button) is False:
        print(button_pressed(Front_button))

        ROBOT.straight(100)
        distance_traveled -= 100

    # Raise the crane slightly to hold the planet
    if (angle_of_crane + raise_angle) <= max_angle:
        Crane_motor.run_target(speed_of_crane, angle_of_crane + raise_angle)
    else:
        Crane_motor.run_target(speed_of_crane, max_angle)

    # Drive back
    ROBOT.straight(distance_traveled)

    return angle_of_crane
