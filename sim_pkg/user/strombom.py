import struct
import numpy as np
import math
import random

# default values (scaled) from the original paper
# sheep parameters
N = 10 # number of sheep
n = 5 # number of nearest neighbors
r_s = 6.5 # shepherd detection distance
r_a = 0.2 # agent to agent interaction distance
p_a = 2.0 # relative strength of repulsion from other agents
c = 1.05 # relative strength of attraction to the n nearest neighbours
p_s = 1.0 # relative strength of repulsion from the shepherd
h = 0.5 # relative strength of proceeding in the previous direction
e = 0.3 # relative strength of angular noise
p = 0.05 # probability of moving per time step while grazing
sheep_speed = 10 # 0.1 # speed of the sheep
st_con = 0.5 # steering constant for the sheep

# shepherd parameters
p_d = r_a*math.sqrt(N) # driving position behind the flock
p_c = r_a # collecting position behind the furthest sheep
beta = math.pi/2 # blind angle behind the shepherd
f_N = 1.0 # if all the sheep are within this distance from their GCM, the shepherd
          # drives the flock to the goal position, or else it collects the flock
shepherd_speed = 25 # 0.25 # speed of the shepherd
sts_con = 0.5 # steering constant for the shepherd

# goal position for the shepherd
goal_x = -7.5/2.0
goal_y = -7.5/2.0
goal_pos = np.array([goal_x, goal_y]) # goal position for the shepherd

# create a function to parse a txt file and assign the values to the user variables
def read_from_txt(filepath):
    global N, n, r_s, r_a, p_a, c, p_s, h, e, p, f_N, sheep_speed, shepherd_speed, st_con, sts_con, goal_x, goal_y
    with open(filepath, "r") as f:
        lines = f.readlines()
        try:
            n = int(lines[0].split(" = ")[1])
            r_s = float(lines[1].split(" = ")[1])
            r_a = float(lines[2].split(" = ")[1])
            p_a = float(lines[3].split(" = ")[1])
            c = float(lines[4].split(" = ")[1])
            p_s = float(lines[5].split(" = ")[1])
            h = float(lines[6].split(" = ")[1])
            e = float(lines[7].split(" = ")[1])
            p = float(lines[8].split(" = ")[1])
            sheep_speed = float(lines[9].split(" = ")[1])
            st_con = float(lines[10].split(" = ")[1])
            f_N = float(lines[11].split(" = ")[1])
            shepherd_speed = float(lines[12].split(" = ")[1])
            sts_con = float(lines[13].split(" = ")[1])
        except:
            pass

def shepherd(robot):
    """
    Function to control the shepherd robot as per the Strombom model.

    Args:
        robot (robot_instance): The robot object to control.
    """
    # read the user variables from the txt file
    read_from_txt("user/strombom_variables.txt")
    # print(f" User Variables: N={N}, n={n}, r_s={r_s}, r_a={r_a}, p_a={p_a}, c={c}, p_s={p_s}, h={h}, e={e}, p={p}, f_N={f_N}, sheep_speed={sheep_speed}, st_con={st_con}, shepherd_speed={shepherd_speed}, sts_con={sts_con}")

    # empty desired vector initialization
    vec_desired = np.array([0.0, 0.0])

    # get robot pose
    pose_t = robot.get_pose()
    if pose_t: # check if pose is valid before using
        
        # calculate current heading
        vec_curr = np.array([np.cos(pose_t[2]), np.sin(pose_t[2])])
        vec_curr = vec_curr/np.linalg.norm(vec_curr) # normalize the vector

        msgs = robot.recv_msg() # receive messages from other robots
        if len(msgs) > 0:
            # check if any other sheep are within the repulsion distance
            # first isolate the sheep messages using the virtual id in the message
            sheep_msgs = [msg for msg in msgs if msg[3] == 0]
            # sort the sheep messages based on distance
            closest_neighbors = []
            for i in range(len(sheep_msgs)):
                dist = np.linalg.norm(np.array([sheep_msgs[i][0], sheep_msgs[i][1]]) - np.array([pose_t[0], pose_t[1]]))
                closest_neighbors.append([dist, i])
            closest_neighbors.sort()

            # check if the closest sheep is within the repulsion distance
            if closest_neighbors[0][0] < 3*r_a:
                robot.set_vel(0.0, 0.0) # stop moving
            else:
                # calculate the Global Center of Mass (GCM) of the sheep flock
                gcm = np.array([0.0, 0.0])
                for i in range(len(sheep_msgs)):
                    gcm = np.add(gcm, np.array([sheep_msgs[i][0], sheep_msgs[i][1]]))
                gcm = gcm/len(sheep_msgs)

                # check if all the sheep are within the f_N distance from the GCM
                all_within_f_N = True
                for i in range(len(sheep_msgs)):
                    dist = np.linalg.norm(np.array([sheep_msgs[i][0], sheep_msgs[i][1]]) - gcm)
                    if dist > f_N:
                        all_within_f_N = False
                        break

                if all_within_f_N:
                    # driving mode
                    # print("Driving mode")
                    # calculate the equation of the line connecting the GCM and the goal position
                    if (gcm[0] - goal_pos[0]) == 0:
                        theta = math.pi/2
                    else:
                        m = (gcm[1] - goal_pos[1])/(gcm[0] - goal_pos[0])
                        c = gcm[1] - m*gcm[0]
                        theta = math.atan(m)
                    # calculate the driving position (p_d) behind the flock    
                    goal_point_1 = np.array([gcm[0] + p_d*np.cos(theta), gcm[1] + p_d*np.sin(theta)])
                    goal_point_2 = np.array([gcm[0] - p_d*np.cos(theta), gcm[1] - p_d*np.sin(theta)])
                    # check which point is closer to the goal point
                    dist_1 = np.linalg.norm(goal_point_1 - goal_pos)
                    dist_2 = np.linalg.norm(goal_point_2 - goal_pos)
                    # set goal point to be the point that is farther from the goal position
                    goal_point = goal_point_1 if dist_1 > dist_2 else goal_point_2

                    # calculate the vector to the goal point
                    vec_goal = np.array([goal_point[0]-pose_t[0], goal_point[1]-pose_t[1]])
                    vec_goal = vec_goal/np.linalg.norm(vec_goal) # normalize the vector
                    # add the vector to the desired vector
                    vec_desired = np.add(vec_desired, vec_goal)
                    vec_desired = vec_desired/np.linalg.norm(vec_desired) # normalize the vector
                else:
                    # collecting mode
                    # print("Collecting mode")
                    # calculate the position of the sheep farthest from the GCM
                    farthest_sheep = np.array([0.0, 0.0])
                    max_dist = 0.0
                    for i in range(len(sheep_msgs)):
                        dist = np.linalg.norm(np.array([sheep_msgs[i][0], sheep_msgs[i][1]]) - gcm)
                        if dist > max_dist:
                            max_dist = dist
                            farthest_sheep = np.array([sheep_msgs[i][0], sheep_msgs[i][1]])

                    # calculate the equation of the line connecting the GCM and the farthest sheep
                    if (gcm[0] - goal_pos[0]) == 0:
                        theta = math.pi/2
                    else:
                        m = (gcm[1] - goal_pos[1])/(gcm[0] - goal_pos[0])
                        c = gcm[1] - m*gcm[0]
                        theta = math.atan(m)
                    # calculate the collecting position (p_c) behind the farthest sheep
                    goal_point_1 = np.array([farthest_sheep[0] + p_c*np.cos(theta), farthest_sheep[1] + p_c*np.sin(theta)])
                    goal_point_2 = np.array([farthest_sheep[0] - p_c*np.cos(theta), farthest_sheep[1] - p_c*np.sin(theta)])
                    # check which point is closer to the gcm
                    dist_1 = np.linalg.norm(goal_point_1 - gcm)
                    dist_2 = np.linalg.norm(goal_point_2 - gcm)
                    # set goal point to be the point that is farther from the gcm
                    goal_point = goal_point_1 if dist_1 > dist_2 else goal_point_2

                    # calculate the vector to the goal point
                    vec_goal = np.array([goal_point[0]-pose_t[0], goal_point[1]-pose_t[1]])
                    vec_goal = vec_goal/np.linalg.norm(vec_goal) # normalize the vector
                    # add the vector to the desired vector
                    vec_desired = np.add(vec_desired, vec_goal)
                    vec_desired = vec_desired/np.linalg.norm(vec_desired) # normalize the vector

                # calculate the error in heading wrt the desired movement direction
                heading_error = math.atan2(np.linalg.det([vec_curr, vec_desired]), np.dot(vec_desired, vec_curr))

                if heading_error > 0:
                    robot.set_vel(shepherd_speed, shepherd_speed + shepherd_speed*sts_con*(abs(heading_error)/np.pi))
                else:
                    robot.set_vel(shepherd_speed + shepherd_speed*sts_con*(abs(heading_error)/np.pi), shepherd_speed)

        # construct message to send to other robots
        pose_t.append(float(robot.virtual_id))
        msg = pose_t
        robot.send_msg(msg)


def sheep(robot):
    """
    Function to control the sheep robot as per the Strombom model.

    Args:
        robot (robot_instance): The robot object to control.
    """
    # read the user variables from the txt file
    read_from_txt("user/strombom_variables.txt")

    # empty desired vector initialization
    vec_desired = np.array([0.0, 0.0])

    # get robot pose
    pose_t = robot.get_pose()
    if pose_t: # check if pose is valid before using

        # calculate current heading
        vec_curr = np.array([np.cos(pose_t[2]), np.sin(pose_t[2])])
        vec_curr = vec_curr/np.linalg.norm(vec_curr) # normalize the vector

        msgs = robot.recv_msg() # receive messages from other robots
        if len(msgs) > 0:
            # check if any other sheep are within the repulsion distance
            # first isolate the sheep messages using the virtual id in the message
            sheep_msgs = [msg for msg in msgs if msg[3] == 0]
            # sort the sheep messages based on distance
            closest_neighbors = []
            for i in range(len(sheep_msgs)):
                dist = np.linalg.norm(np.array([sheep_msgs[i][0], sheep_msgs[i][1]]) - np.array([pose_t[0], pose_t[1]]))
                closest_neighbors.append([dist, i])
            closest_neighbors.sort()

            # calculate the LCM of sheep within the repulsion distance
            sheep_within_r = 0
            for i in range(len(closest_neighbors)):
                if closest_neighbors[i][0] < r_a:
                    sheep_within_r += 1
            if sheep_within_r > 0:
                lcm = np.array([0.0, 0.0])
                for i in range(sheep_within_r):
                    lcm = np.add(lcm, np.array([sheep_msgs[closest_neighbors[i][1]][0], sheep_msgs[closest_neighbors[i][1]][1]]))
                lcm = lcm/sheep_within_r

                # calculate the vector to the LCM
                vec_repulsion = - np.array([lcm[0]-pose_t[0], lcm[1]-pose_t[1]]) # vector pointing away from the LCM
                vec_repulsion = vec_repulsion/np.linalg.norm(vec_repulsion) # normalize the vector
                # add the vector to the desired vector
                vec_desired = np.add(vec_desired, p_a*vec_repulsion)
                vec_desired = vec_desired/np.linalg.norm(vec_desired) # normalize the vector
                
            # calculate the distance to the shepherd
            # first isolate the shepherd message using the virtual id in the message
            shepherd_msg = [msg for msg in msgs if msg[3] == 1]
            shepherd_msg = shepherd_msg[0] if len(shepherd_msg) > 0 else None # pick the first message if multiple messages are received
            if shepherd_msg is not None:
                # calculate the distance to the shepherd
                dist_shepherd = np.linalg.norm(np.array([shepherd_msg[0], shepherd_msg[1]]) - np.array([pose_t[0], pose_t[1]]))
                if dist_shepherd < r_s:
                    vec_shepherd = - np.array([shepherd_msg[0]-pose_t[0], shepherd_msg[1]-pose_t[1]]) # vector pointing away from the shepherd
                    vec_shepherd = vec_shepherd/np.linalg.norm(vec_shepherd) # normalize the vector

                    # add the vector to the desired vector
                    vec_desired = np.add(vec_desired, p_s*vec_shepherd)
                    vec_desired = vec_desired/np.linalg.norm(vec_desired) # normalize the vector
    
                    # calculate the local center of mass (LCM) of the n nearest neighbors
                    # calculate the LCM
                    lcm_num = min(n, len(closest_neighbors))
                    lcm = np.array([0.0, 0.0])
                    for i in range(lcm_num):
                        lcm = np.add(lcm, np.array([sheep_msgs[closest_neighbors[i][1]][0], sheep_msgs[closest_neighbors[i][1]][1]]))
                    lcm = lcm/n

                    # calculate the vector to the LCM
                    vec_attraction = np.array([lcm[0]-pose_t[0], lcm[1]-pose_t[1]])
                    vec_attraction = vec_attraction/np.linalg.norm(vec_attraction)
                    # add the vector to the desired vector
                    vec_desired = np.add(vec_desired, c*vec_attraction)
                    vec_desired = vec_desired/np.linalg.norm(vec_desired)

        # move only if the sheep is prompted to move
        if vec_desired[0] != 0.0 and vec_desired[1] != 0.0:
            # calculate the error in heading wrt the desired movement direction
            heading_error = math.atan2(np.linalg.det([vec_curr, vec_desired]), np.dot(vec_desired, vec_curr))

            if heading_error > 0:
                robot.set_vel(sheep_speed, sheep_speed + sheep_speed*st_con*(abs(heading_error)/np.pi)) # turn left
            else:
                robot.set_vel(sheep_speed + sheep_speed*st_con*(abs(heading_error)/np.pi), sheep_speed) # turn right
        else:
            robot.set_vel(0.0, 0.0) # stop moving
        
        # construct message to send to other robots
        pose_t.append(float(robot.virtual_id))
        msg = pose_t
        robot.send_msg(msg)    

def usr(robot):
    if robot.id == 0:
        robot.set_led(0, 0, 255)
        while True:
            shepherd(robot)
    else:
        robot.set_led(0, 255, 0)
        while True:
            sheep(robot)