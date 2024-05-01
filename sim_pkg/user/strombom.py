import struct
import numpy as np
import math
import random

# default values found through trial and error
d = 0.3 # repulsion distance threshold
r = 6.0 # repulsion constant
a = 0.8 # attraction constant
k = 1.2 # heading constant
m = 20.0 # migration constant
s = 0.55 # steering constant
n = 5 # number of neighbors

# create a function to parse a txt file and assign the values to the user variables
def read_from_txt(filepath):
    global d, r, a, k, m, s, n
    with open(filepath, "r") as f:
        lines = f.readlines()
        try:
            d = float(lines[0].split(" = ")[1])
            r = float(lines[1].split(" = ")[1])
            a = float(lines[2].split(" = ")[1])
            k = float(lines[3].split(" = ")[1])
            m = float(lines[4].split(" = ")[1])
            s = float(lines[5].split(" = ")[1])
            n = int(lines[6].split(" = ")[1])
        except:
            pass

def shepherd(robot):
    """
    Function to control the shepherd robot as per the Strombom model.

    Args:
        robot (robot_instance): The robot object to control.
    """

    pass

def sheep(robot):
    """
    Function to control the sheep robot as per the Strombom model.

    Args:
        robot (robot_instance): The robot object to control.
    """
    pass

def usr(robot):
    if robot.id == 0:
        robot.set_led(0, 0, 255)
        shepherd(robot)
    else:
        robot.set_led(0, 255, 0)
        sheep(robot)