import argparse
import json
import time
import multiprocessing as mp
import threading
from bootloader import Bootloader
from sim import Simulator
from gui import GUI

import cv2

# create a function to parse a txt file and assign the values to the user variables
def flocking_write_to_txt(filepath, d, r, a, k, m, s, n):
    with open(filepath, "w") as f:
        f.write(f"d = {d}\n")
        f.write(f"r = {r}\n")
        f.write(f"a = {a}\n")
        f.write(f"k = {k}\n")
        f.write(f"m = {m}\n")
        f.write(f"s = {s}\n")
        f.write(f"n = {n}\n")

        f.close()

def strombom_write_to_txt(filepath, n, r_s, r_a, p_a, c, p_s, h, e, p, sheep_speed, st_con, f_N, shepherd_speed, sts_con, goal_x, goal_y, k):
    with open(filepath, "w") as f:
        f.write(f"n = {n}\n")
        f.write(f"r_s = {r_s}\n")
        f.write(f"r_a = {r_a}\n")
        f.write(f"p_a = {p_a}\n")
        f.write(f"c = {c}\n")
        f.write(f"p_s = {p_s}\n")
        f.write(f"h = {h}\n")
        f.write(f"e = {e}\n")
        f.write(f"p = {p}\n")
        f.write(f"sheep_speed = {sheep_speed}\n")
        f.write(f"st_con = {st_con}\n")
        f.write(f"f_N = {f_N}\n")
        f.write(f"shepherd_speed = {shepherd_speed}\n")
        f.write(f"sts_con = {sts_con}\n")
        f.write(f"goal_x = {goal_x}\n")
        f.write(f"goal_y = {goal_y}\n")
        f.write(f"k = {k}\n")

        f.close()

def strombom_create_trackbars():
    # create trackbars to adjust the values of the user variables
    cv2.namedWindow("User Variables")
    cv2.createTrackbar("n", "User Variables", 0, 20, lambda x: None)
    cv2.createTrackbar("r_s", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("r_a", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("p_a", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("c", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("p_s", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("h", "User Variables", 0, 10, lambda x: None)
    cv2.createTrackbar("e", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("p", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("sheep_speed", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("st_con", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("f_N", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("shepherd_speed", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("sts_con", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("goal_x", "User Variables", 0, 150, lambda x: None)
    cv2.createTrackbar("goal_y", "User Variables", 0, 150, lambda x: None)
    cv2.createTrackbar("k", "User Variables", 0, 100, lambda x: None)

    # set the default values of the user variables
    cv2.setTrackbarPos("n", "User Variables", 5)
    cv2.setTrackbarPos("r_s", "User Variables", 65)
    cv2.setTrackbarPos("r_a", "User Variables", 4)
    cv2.setTrackbarPos("p_a", "User Variables", 20)
    cv2.setTrackbarPos("c", "User Variables", 6)
    cv2.setTrackbarPos("p_s", "User Variables", 10)
    cv2.setTrackbarPos("h", "User Variables", 5)
    cv2.setTrackbarPos("e", "User Variables", 30)
    cv2.setTrackbarPos("p", "User Variables", 5)
    cv2.setTrackbarPos("sheep_speed", "User Variables", 50)
    cv2.setTrackbarPos("st_con", "User Variables", 40)
    cv2.setTrackbarPos("f_N", "User Variables", 20)
    cv2.setTrackbarPos("shepherd_speed", "User Variables", 75)
    cv2.setTrackbarPos("sts_con", "User Variables", 5)
    cv2.setTrackbarPos("goal_x", "User Variables", 37)
    cv2.setTrackbarPos("goal_y", "User Variables", 37)
    cv2.setTrackbarPos("k", "User Variables", 12)

    while True:
        # Get the current value of the trackbar
        n = cv2.getTrackbarPos("n", "User Variables")
        r_s = cv2.getTrackbarPos("r_s", "User Variables")
        r_s = r_s/10
        r_a = cv2.getTrackbarPos("r_a", "User Variables")
        r_a = r_a/10
        p_a = cv2.getTrackbarPos("p_a", "User Variables")
        p_a = p_a/10
        c = cv2.getTrackbarPos("c", "User Variables")
        c = c/10
        p_s = cv2.getTrackbarPos("p_s", "User Variables")
        p_s = p_s/10
        h = cv2.getTrackbarPos("h", "User Variables")
        h = h/10
        e = cv2.getTrackbarPos("e", "User Variables")
        e = e/100
        p = cv2.getTrackbarPos("p", "User Variables")
        p = p/100
        sheep_speed = cv2.getTrackbarPos("sheep_speed", "User Variables")
        st_con = cv2.getTrackbarPos("st_con", "User Variables")
        st_con = st_con/10
        f_N = cv2.getTrackbarPos("f_N", "User Variables")
        f_N = f_N/10
        shepherd_speed = cv2.getTrackbarPos("shepherd_speed", "User Variables")
        sts_con = cv2.getTrackbarPos("sts_con", "User Variables")
        sts_con = sts_con/10
        goal_x = cv2.getTrackbarPos("goal_x", "User Variables")
        goal_x = (goal_x/10) - 7.5 
        goal_y = cv2.getTrackbarPos("goal_y", "User Variables")
        goal_y = (goal_y/10) - 7.5
        k = cv2.getTrackbarPos("k", "User Variables")
        k = k/10

        strombom_write_to_txt("user/strombom_variables.txt", n, r_s, r_a, p_a, c, p_s, h, e, p, sheep_speed, st_con, f_N, shepherd_speed, sts_con, goal_x, goal_y, k)

        cv2.waitKey(1)

def flocking_create_trackbars():
    # create trackbars to adjust the values of the user variables
    cv2.namedWindow("User Variables")
    cv2.createTrackbar("d", "User Variables", 0, 10, lambda x: None)
    cv2.createTrackbar("r", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("a", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("k", "User Variables", 0, 1000, lambda x: None)
    cv2.createTrackbar("m", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("s", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("n", "User Variables", 0, 10, lambda x: None)

    # set the default values of the user variables
    cv2.setTrackbarPos("d", "User Variables", 6)
    cv2.setTrackbarPos("r", "User Variables", 80)
    cv2.setTrackbarPos("a", "User Variables", 8)
    cv2.setTrackbarPos("k", "User Variables", 12)
    cv2.setTrackbarPos("m", "User Variables", 25)
    cv2.setTrackbarPos("s", "User Variables", 30)
    cv2.setTrackbarPos("n", "User Variables", 5)
    
    while True:
        # Get the current value of the trackbar
        d = cv2.getTrackbarPos("d", "User Variables")
        d = d/10
        r = cv2.getTrackbarPos("r", "User Variables")
        r = r/10
        a = cv2.getTrackbarPos("a", "User Variables")
        a = a/10
        k = cv2.getTrackbarPos("k", "User Variables")
        k = k/10
        m = cv2.getTrackbarPos("m", "User Variables")
        s = cv2.getTrackbarPos("s", "User Variables")
        s = s/100
        n = cv2.getTrackbarPos("n", "User Variables")

        flocking_write_to_txt("user/flocking_variables.txt", d, r, a, k, m, s, n)
        
        cv2.waitKey(1)

def run_threads(bootloader, simulator, num_robots): 
    '''
    Starts a thread that calls bootloader launch for each robot 
    '''
    barrier = threading.Barrier(num_robots)
    threads = [threading.Thread(target=bootloader.launch, args=(barrier, i, simulator.swarm[i].a_ids)) for i in range(num_robots)]
    for thread in threads:
        thread.start()

def main(userfile, config_data, initfile, run_number, trial_number, calibrate_flocking=False, calibrate_strombom=False):
    '''
    Runs the robots, simulator, and GUI in parallel. 
    If the program is interrupted, terminate all processes cleanly.
    '''
    print(f"Simulation {run_number} Trial {trial_number} initiated.")

    # Number of robots
    num_robots = config_data["NUMBER_OF_ROBOTS"]
    # Boolean switch determines whether or not to run the GUI
    vis = config_data["USE_VIS"]

    # Initialize classes and run processes simultaneously
    bootloader = Bootloader(userfile, config_data)
    simulator = Simulator(config_data, initfile)
    if vis == 1:
        gui = GUI(config_data, trial_number)
    elif vis == 0:
        gui = None

    # Start simulator process (this process also runs the gui)
    s_proc = mp.Process(target=simulator.launch, args=(vis, gui))
    s_proc.start()
    time.sleep(2) # TODO: This value may need to be adjusted
                   # The clients connect extremely quickly and may connect before the simulator server starts receiving data -- a large pause here is needed to prevent important commands at the beginning of the user code from being missed

    # TODO: for python2 compatibility on the bootloader, need to specify path to py2 and py3 interpreter on user's machine -- pool_context.set_executable(...)  
    r_proc = mp.Process(target=run_threads, args=(bootloader, simulator, num_robots))
    r_proc.start()

    # start the thread that creates the trackbars for flocking calibration
    if calibrate_flocking:
        t_proc = mp.Process(target=flocking_create_trackbars)
        t_proc.start()

    # start the thread that creates the trackbars for strombom calibration
    if calibrate_strombom:
        t_proc = mp.Process(target=strombom_create_trackbars)
        t_proc.start()

    # Wait for simulator to complete or terminate cleanly
    try:
        s_proc.join() # Wait for simulator process to complete
        r_proc.terminate() # Terminate the robot process after the simulator process is finished
        if calibrate_flocking: t_proc.terminate() # Terminate the trackbar process after the simulator process is finished
        if calibrate_strombom: t_proc.terminate()
        print(f"Simulation {run_number} Trial {trial_number} completed.")

    except KeyboardInterrupt:
        s_proc.terminate()
        r_proc.terminate()
        if calibrate_flocking: t_proc.terminate()
        if calibrate_strombom: t_proc.terminate()
        print(f"User interrupted: Simulation {run_number} Trial {trial_number} terminated.")

if __name__ == '__main__':
    # Parse command line arguments to obtain the paths to the required files
    parser = argparse.ArgumentParser(description="Run the robots, simulator, and GUI")
    parser.add_argument("-b", "--batchfile", type=str, help="Path to user code file", required=True)
    parser.add_argument("-cf", "--calibrate_flocking", type=bool, help="Open Trackbars for calibration", required=False, default=False)
    parser.add_argument("-cs", "--calibrate_strombom", type=bool, help="Open Trackbars for strombom calibration", required=False, default=False)
    args = parser.parse_args()

    # Unpack batchfile data
    with open("user/" + args.batchfile, 'r') as bfile:
        batch_config = json.loads(bfile.read())
    bfile.close()

    calibrate_flocking = args.calibrate_flocking
    calibrate_strombom = args.calibrate_strombom

    num_runs = batch_config["NUM_RUNS"]

    for i in range(1, num_runs + 1):
        if f"TRIALS_{i}" in batch_config:
            num_trials = batch_config[f"TRIALS_{i}"]
        else:
            num_trials = batch_config["DEFAULT_TRIALS"]

        for trial in range(1, num_trials + 1):
            if f"USER_{i}" in batch_config:
                userfile = batch_config[f"USER_{i}"]
            else:
                userfile = batch_config["DEFAULT_USER"]

            if f"CONFIG_{i}" in batch_config:
                configfile = batch_config[f"CONFIG_{i}"]
            else:
                configfile = batch_config["DEFAULT_CONFIG"]

            # Unpack configuration data dictionary
            with open("user/" + configfile, 'r') as cfile:
                config_data = json.loads(cfile.read())
            cfile.close()

            if f"INIT_{i}" in batch_config:
                initfile = batch_config[f"INIT_{i}"]
            elif "DEFAULT_INIT" in batch_config:
                initfile = batch_config["DEFAULT_INIT"]
            else:
                initfile = None
            
            # Run main function
            main(userfile, config_data, initfile, i, trial, calibrate_flocking, calibrate_strombom)