from pybricks.hubs import EV3Brick as brick
from pybricks.parameters import Button
from numpy import save
from pybricks.tools import wait
from pybricks.ev3devices import ColorSensor

saved_colours =  open("savedColours.txt", "w")
def Calibrate_Colours(colours,sensor):
    """
    Enter a calibraton mode where the user must press a button on the brick and set the colour value for each individual colour reading.

    Returns a dictionary of colours and saves to txt file
    """
    calibrated_colours = colours.copy()

    for label,colour in colours.items():
        print("set colour for: "+label)
        running = True
        while running:
            if brick.buttons.pressed(): #if any button is pressed
                running = False
        selected_colour = colour#sensor.color()
        #^Uncomment for testing^
        calibrated_colours[i] = selected_colour
        saved_colours.write(str(selected_colour)+"\n")
    saved_colours.close()
    print(calibrated_colours)
    return calibrated_colours

    