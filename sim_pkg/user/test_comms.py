import struct
import numpy as np
import math
import random

d = 0.5 # repulsion distance threshold
r = 10.0 # repulsion constant
a = 20.0 # attraction constant
s = 1.0 # steering constant

def message(robot):
    if robot.id == 0:
        msg = robot.recv_msg()
        if len(msg) > 0:
            print(f"{msg =}")
        else:
            print("No msg")
    else:
        robot.send_msg(f"{robot.id}")

def center(robot):
    """
    Function to congregagte all robots to the
    center of the arena.

    Args:
        robot (_type_): robot instance
    """

    # empty desired vector initialization
    vec_desired = np.array([0.0, 0.0])
    
    # get robot pose
    pose_t = robot.get_pose()
    if pose_t: # check if pose is valid before using

        # calculate current heading
        vec_curr = np.array([np.cos(pose_t[2]), np.sin(pose_t[2])])
        vec_curr = vec_curr/np.linalg.norm(vec_curr) # normalize the vector

        # calculate the vector to the center
        vec_center = np.array([0.0-pose_t[0], 0.0-pose_t[1]])
        vec_center = vec_center/np.linalg.norm(vec_center) # normalize the vector
        vec_desired = np.add(vec_desired, a*vec_center) # add the vector to the desired vector
        vec_desired = vec_desired/np.linalg.norm(vec_desired) # normalize the vector

        msgs = robot.recv_msg()
        if len(msgs) > 0:
            # find calculate the distance from the closest neighbor
            min_dist = 1e10
            closest = 0
            for i in range(len(msgs)):
                dist = math.dist([pose_t[0], pose_t[1]], [msgs[i][0], msgs[i][1]])
                if dist < min_dist:
                    min_dist = dist
                    closest = i

            # if the distance is less than 1.0, calculate and add the repulsion vector
            if min_dist < d:
                vec_rep = np.array([pose_t[0]-msgs[closest][0], pose_t[1]-msgs[closest][1]])
                vec_rep = vec_rep/np.linalg.norm(vec_rep) # normalize the vector
                vec_desired = np.add(vec_desired, r*vec_rep)
                vec_desired = vec_desired/np.linalg.norm(vec_desired) # normalize the vector

        # calculate the error in heading wrt the desired movement direction
        heading_error = math.atan2(np.linalg.det([vec_curr, vec_desired]), np.dot(vec_desired, vec_curr))

        if heading_error > 0:
            robot.set_vel(10, 10 + 10*s*(abs(heading_error)/np.pi)) # turn left
        else:
            robot.set_vel(10 + 10*s*(abs(heading_error)/np.pi), 10)

        # send pose message
        robot.send_msg(pose_t)


def usr(robot):
    robot.set_led(0, 0, 255)
    while True:
        center(robot)