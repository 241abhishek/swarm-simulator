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
def write_to_txt(filepath, d, r, a, k, m, s, n):
    with open(filepath, "w") as f:
        f.write(f"d = {d}\n")
        f.write(f"r = {r}\n")
        f.write(f"a = {a}\n")
        f.write(f"k = {k}\n")
        f.write(f"m = {m}\n")
        f.write(f"s = {s}\n")
        f.write(f"n = {n}\n")

def create_trackbars():
    # create trackabars to adjust the values of the user variables
    cv2.namedWindow("User Variables")
    cv2.createTrackbar("d", "User Variables", 0, 10, lambda x: None)
    cv2.createTrackbar("r", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("a", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("k", "User Variables", 0, 1000, lambda x: None)
    cv2.createTrackbar("m", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("s", "User Variables", 0, 100, lambda x: None)
    cv2.createTrackbar("n", "User Variables", 0, 10, lambda x: None)

    # set the default values of the user variables
    cv2.setTrackbarPos("d", "User Variables", 4)
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

        write_to_txt("user/flocking_variables.txt", d, r, a, k, m, s, n)
        
        cv2.waitKey(1)

def run_threads(bootloader, simulator, num_robots): 
    '''
    Starts a thread that calls bootloader launch for each robot 
    '''
    barrier = threading.Barrier(num_robots)
    threads = [threading.Thread(target=bootloader.launch, args=(barrier, i, simulator.swarm[i].a_ids)) for i in range(num_robots)]
    for thread in threads:
        thread.start()

def main(userfile, config_data, initfile, run_number, trial_number, calibrate):
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

    # start the thread that creates the trackbars
    if calibrate:
        t_proc = mp.Process(target=create_trackbars)
        t_proc.start()

    # Wait for simulator to complete or terminate cleanly
    try:
        s_proc.join() # Wait for simulator process to complete
        r_proc.terminate() # Terminate the robot process after the simulator process is finished
        if calibrate: t_proc.terminate() # Terminate the trackbar process after the simulator process is finished
        print(f"Simulation {run_number} Trial {trial_number} completed.")

    except KeyboardInterrupt:
        s_proc.terminate()
        r_proc.terminate()
        if calibrate: t_proc.terminate()
        print(f"User interrupted: Simulation {run_number} Trial {trial_number} terminated.")

if __name__ == '__main__':
    # Parse command line arguments to obtain the paths to the required files
    parser = argparse.ArgumentParser(description="Run the robots, simulator, and GUI")
    parser.add_argument("-b", "--batchfile", type=str, help="Path to user code file", required=True)
    parser.add_argument("-c", "--calibrate", type=bool, help="Open Trackbars for calibration", required=False, default=False)
    args = parser.parse_args()

    # Unpack batchfile data
    with open("user/" + args.batchfile, 'r') as bfile:
        batch_config = json.loads(bfile.read())
    bfile.close()

    calibrate = args.calibrate

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
            main(userfile, config_data, initfile, i, trial, calibrate)