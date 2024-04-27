import struct
import numpy as np
import math
import random

def message(robot):
    if robot.id == 0:
        robot.send_msg("hi")
    else:
        msg = robot.recv_msg()
        if len(msg) > 0:
            print(f"{msg =}")
        else:
            print("No msg")


def usr(robot):
    while True:
        if robot.id == 0:
            robot.set_led(255, 0, 0)
            message(robot)
        if robot.id == 1:
            robot.set_led(0, 255, 0)
            message(robot)