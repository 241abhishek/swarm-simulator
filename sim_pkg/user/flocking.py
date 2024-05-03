import struct
import numpy as np
import math
import random

# default values found through trial and error
d = 0.4 # repulsion distance threshold
r = 8.0 # repulsion constant
a = 0.8 # attraction constant
k = 1.2 # heading constant
m = 25.0 # migration constant
s = 0.3 # steering constant
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

def center(robot):
    """
    Function to implement the flocking algorithm.

    Args:
        robot (_type_): robot instance
    """

    # read the user variables from the txt file
    read_from_txt("user/flocking_variables.txt")
    print(f"{d =}, {r =}, {a =}, {k =}, {m =}, {s =}, {n =}")

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
        # calculate the distance to the center
        dist_center = np.linalg.norm(vec_center)
        vec_desired = np.add(vec_desired, m*(dist_center**3)*vec_center) # add the vector to the desired vector
        vec_desired = vec_desired/np.linalg.norm(vec_desired) # normalize the vector

        msgs = robot.recv_msg()
        if len(msgs) > 0:
            # calculate the average heading of all neighbors
            avg_heading = 0.0
            for i in range(len(msgs)):
                avg_heading += msgs[i][2]
            avg_heading = avg_heading/len(msgs)
            vec_heading = np.array([np.cos(avg_heading), np.sin(avg_heading)]) # conver to vector form
            vec_heading = vec_heading/np.linalg.norm(vec_heading) # normalize the vector
            vec_desired = np.add(vec_desired, k*vec_heading) # add the vector to the desired vector
            vec_desired = vec_desired/np.linalg.norm(vec_desired) # normalize the vector

            # find the COM of n closest neighbors
            com = np.array([0.0, 0.0])
            closest_neighbors = []
            min_dist = 1e10
            closest = 0
            for i in range(len(msgs)):
                dist = math.dist([pose_t[0], pose_t[1]], [msgs[i][0], msgs[i][1]])
                closest_neighbors.append([dist, i])
                if dist < min_dist:
                    min_dist = dist
                    closest = i

            # sort the neighbors based on distance
            closest_neighbors.sort()
            num_neighbors = min(n, len(closest_neighbors))
            for i in range(num_neighbors):
                com = np.add(com, [msgs[closest_neighbors[i][1]][0], msgs[closest_neighbors[i][1]][1]])
            com = com/n

            # calculate the attraction vector
            vec_att = np.array([com[0]-pose_t[0], com[1]-pose_t[1]])
            vec_att = vec_att/np.linalg.norm(vec_att)   # normalize the vector
            vec_desired = np.add(vec_desired, a*vec_att)
            vec_desired = vec_desired/np.linalg.norm(vec_desired) # normalize the vector

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