
"""A place to store test functions
    """


def test_crane():
    Crane_motor.reset_angle(0)
    max_angle = crane_movement(Crane_motor, 1, 50)
    min_angle = crane_movement(Crane_motor, -1, 50)

    crane_pickup(Crane_motor, TRUCK, Front_button, -1000, max_angle, min_angle)
    crane_pickup(Crane_motor, TRUCK, Front_button,
                 max_angle/2, max_angle, min_angle)


def test_warehouse():
    warehouse_drive(
        light_sensor, TRUCK, (3, 3, 2), (41, 36, 4))


def test_crane_pickup():
    # If on the yellow line on a warehouse zone
    crane_pickup(Crane_motor, light_sensor, TRUCK,
                 Front_button, -30, (3, 3, 2), (41, 36, 4))


def test_emergency_mode():
    list_of_colours, colour_background, warehouse_colour, warehouse_line = startup()
    reversed_list = list_of_colours[::-1]
    pickupstatus = True
    while pickupstatus is True:
        pickupstatus = detect_item_fail(pickupstatus)
    drive(reversed_list, colour_background,
          warehouse_colour, warehouse_line, EV3)
