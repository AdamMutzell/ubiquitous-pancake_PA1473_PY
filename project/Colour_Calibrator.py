saved_colours =  open("savedColours.txt", "w")

from numpy import save
from pybricks.hubs import MoveHub
from pybricks.tools import wait

from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Button

hub = MoveHub()
def Calibrate_Colours(colours,sensor):
    for colour, i in enumerate(colours()):
        print("set colour for: ",colour)
        while hub.button.pressed() == False:
            wait(1)
        selected_colour = colour#sensor.color()
        colours[i] = selected_colour
        saved_colours.write(selected_colour,"\n")
    saved_colours.close()
    print(colours)
    return colours