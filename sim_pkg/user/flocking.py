import struct
import numpy as np
import math
import random

center = [0,0] # migration point
m = 2.0 # migration constant
k = 10.0 # heading constant
r = 1.0 # repulsion constant
a = 0.8 # attraction constant
s = 3.0 # steering constant

def flocking(robot, neighbors_headings, neighbors_pos_x, neighbors_pos_y):
    """
    Function to calculate and set robot velocity as per
    the flocking algorithm.
    """

    vec_total = np.array([0.0,0.0]) # initialize empty vector

    pose_t = robot.get_pose()
    if pose_t:  # check pose is valid before using
        vec_curr = np.array([np.cos(pose_t[2]), np.sin(pose_t[2])]) # current heading

        dist_mig = math.dist([center[0], center[1]], [pose_t[0], pose_t[1]]) # calculate distance from the migration point

        vec_ic = np.array([center[0]-pose_t[0], center[1]-pose_t[1]]) # vector to migration point
        vec_ic_unit = vec_ic/np.linalg.norm(vec_ic) # unit migration vector
        vec_total = np.add(vec_total,m*dist_mig*vec_ic_unit) # add the migration vector to vec_total

        msgs = robot.recv_msg()
        if len(msgs) > 0: # check for message
            pose_rxed = struct.unpack('fff', msgs[0][:12])

            dist = math.dist([pose_rxed[0], pose_rxed[1]], [pose_t[0], pose_t[1]]) # calculate distance from neighbor

            if dist < 1.0:
                # append the postitions to the neighbors_pos lists
                neighbors_pos_x.append(pose_rxed[0])
                neighbors_pos_y.append(pose_rxed[1])

            if dist < 5.0:
                neighbors_headings.append(pose_rxed[2]) # append the heading to the neighbors_heading lists

            if len(neighbors_pos_x) > 2: # check for minimum number of messages received
                neigh_avg_pose = [np.average(neighbors_pos_x[-2:]),np.average(neighbors_pos_y[-2:])] # calculate approximate position of neighbors
                vec_att = np.array([neigh_avg_pose[0]-pose_t[0], neigh_avg_pose[1]-pose_t[1]]) # calculate attraction vector
                vec_total = np.add(vec_total,a*vec_att) # add weighted attraction vector to vec_total

            if len(neighbors_headings) > 1: # check for minimum number of messages received
                neigh_avg_heading = np.average(neighbors_headings[-1:]) # calculate approximate heading of neighbors
                vec_heading = np.array([np.cos(neigh_avg_heading), np.sin(neigh_avg_heading)]) # average heading in vector form
                vec_total = np.add(vec_total,k*vec_heading) # add weighted heading vector to vec_total

            if dist < 0.5:
                vec_rep = np.array([pose_t[0]-pose_rxed[0], pose_t[1]-pose_rxed[1]]) # calculate repulsion vector
                vec_total = np.add(vec_total,r*(1.0/vec_rep)) # add weighted repulsion vector to vec_total

        heading_error = math.atan2(np.linalg.det([vec_curr,vec_total]),np.dot(vec_total,vec_curr)) # compute the error in heading wrt the desired movement direction
        
        if heading_error > 0: 
            robot.set_vel(10, 10 + s*(abs(heading_error)/np.pi)) # turn left
        elif heading_error < 0:
            robot.set_vel( 10 + s*(abs(heading_error)/np.pi), 10) # turn right

        robot.send_msg(struct.pack('fff', pose_t[0], pose_t[1], pose_t[2]))  # send pose x,y,theta in message

def usr(robot):
    neighbors_pos_x = [] # empty list to record x positions of neighbors
    neighbors_pos_y = [] # empty list to record y positions of neighbors
    neighbors_headings = [] # empty list to record heading of neighbors
    while True:
        robot.set_led(100,0,0) # set color to red
        flocking(robot, neighbors_headings, neighbors_pos_x, neighbors_pos_y)