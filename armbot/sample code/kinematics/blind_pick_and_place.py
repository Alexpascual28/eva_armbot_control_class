# -*- coding: utf-8 -*-
"""
Created on Mon May  1 17:23:18 2023

@title: Lab Session 7 Task 3: Implement Picking-Up Motion
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

# =============================================================================

# Now that you can place the arm over top of the targets, you can implement motion 
# to pick it up.  In most industrial robots, this motion is prescribed based on the 
# known position of the target, and you can most easily just have the arm move to a 
# pre-programmed height so that it is low enough for your gripper to grab the object, 
# and then close the gripper fingers.  You can also if needed refine the position 
# of the object as the arm is lowered.  Then, close the gripper, raise the arm, and 
# move the object using the arm to a new location, then release it.

# =============================================================================

# 1. import all the modules/libraries for Python that are needed for your code, including NumPy, SymPy, MatPlotLib 
#    (if you want to use plotting of your kinematics as before), the evasdk module, and the aravis module.
import numpy as np
import matplotlib.pyplot
from armbot import Kinematics

from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset, solve, simplify, S, diff, det, erf, log, sqrt, pi, sin, cos, tan, atan2, init_printing

from evasdk import Eva
from json import dumps
 
import time

# 2. Initialize your kinematics functions and variables including the constants (dimensions, etc.) for the EVA robot arm.

# Initializes main kinematics class to assist with robot movement
def initialize_kinematics():
    # Dimensions of each link of the robot in metres
    arm_dimensions = [0.187, 0.096, 0.205, 0.124, 0.167, 0.104];
    
    # Robot initial pose (in joint angles)
    initial_pose = [0, 1, -2.5, 0, -1.6, 0] # [0, 1, -2.5, 0, -1.6, 0] # or [0,0,0,0,0,0]; # or [-pi/2,-pi/4,3*pi/4,0,pi/2,0];
    simulate = False # Whether the calculations should be displayed as a graphical simulation  
    
    # Variables to control calculation speed and accuracy
    dp_threshold = 0.01 # Calculations will stop when dp is lower than the threshold
    step_size = 0.01 # In each step, the robot will move this amount
    theta_max_step = 0.5 # Each joint won't rotate more than this angle in radians
    pause = 0.0000001 # Time the plot will be displayed
    
    # Variables to control how the plot is displayed
    plot_dimensions = [[-0.5, 0.5], # x axis limits in metres (min, max)
                       [-0.5, 0.5], # y axis limits in metres (min, max)
                       [0, 0.5]]    # z axis limits in metres (min, max)
    camera_view = [30, 45] # elev, azim (camera position and rotation)
    
    # All values are written to the Kinematics class
    kn = Kinematics(arm_dimensions, initial_pose, simulate, dp_threshold, step_size, theta_max_step, pause, plot_dimensions, camera_view);
    return kn

# 3. Initialize the arm and the camera as in the previous labs and start the camera capturing frames and showing them 
#    in an OpenCV window.

# Initializes robot arm module with specified conexion details
def initialize_arm():
    print("\nInitialising EVA robot arm...")
    
    # Details to connect to the robot arm, more info here: 
    # https://wiki.york.ac.uk/display/TSS/Network+Details+and+Credentials+for+the+EVA+Arms+and+Network+Cameras 
    arm_hostname = "evatrendylimashifterpt410"
    arm_ip = "144.32.152.105"
    token = "1462980d67d58cb7eaabe8790d866609eb97fd2c"
  
    # Create an “eva” object with these parameters to connect to the arm itself on the network
    eva = Eva(arm_ip, token)
    print("Done.\n")
    return eva

def main():
    # Connect to the arm, camera and kinematics modules
    kn = initialize_kinematics();
    eva = initialize_arm();
    
    # Start block
    try:     
        # 4. Move the arm to a starting position, ideally near where your target is located so that your camera can see it, 
        #    and initialize the state of the robot arm to have no object in its gripper and to move to pick up.
        target_position = [0, -0.2, 0.3];
        current_pose = kn.calculate_joint_angles(target_position, position_type = "absolute")
        
        with eva.lock():
          eva.control_wait_for_ready()
          eva.control_go_to(current_pose)
        
        # Open the gripper (to get rid of whatever is being gripped and test the gripper)
        with eva.lock():
           eva.control_wait_for_ready()
           eva.gpio_set('ee_d0', False)
           eva.gpio_set('ee_d1', True)
           
        time.sleep(0.2)
        
        # Close your gripper (or open it, depending on the design you have made), 
        # you need to set ee_d1 to a low logic state and ee_d2 to a high logic state
        with eva.lock():
           eva.control_wait_for_ready()
           eva.gpio_set('ee_d1', False)
           eva.gpio_set('ee_d0', True)

        target_position = [0, 0, 0.05]
           
        # Move the robot
        current_pose = kn.calculate_joint_angles(target_position, position_type = "relative")
        with eva.lock():
            eva.control_wait_for_ready()
            eva.control_go_to(current_pose)
                        
        # Open gripper
        with eva.lock():
            eva.control_wait_for_ready()
            eva.gpio_set('ee_d0', False)
            eva.gpio_set('ee_d1', True)
                
        # Move towards the object
        pick_up_position = [0, 0, -0.05] # Object pick up distance, relative to the camera
                
        current_pose = kn.calculate_joint_angles(pick_up_position, position_type = "relative")
        with eva.lock():
            eva.control_wait_for_ready()
            eva.control_go_to(current_pose)
                  
        # Close gripper
        with eva.lock():
            eva.control_wait_for_ready()
            eva.gpio_set('ee_d1', False)
            eva.gpio_set('ee_d0', True)
                   
        # Move up
        pick_up_position = [0, 0, 0.3] # Object pick up distance, relative to the camera
                
        current_pose = kn.calculate_joint_angles(pick_up_position, position_type = "relative")
        with eva.lock():
            eva.control_wait_for_ready()
            eva.control_go_to(current_pose)
                   
        # Move to target drop location
        target_drop_location = [0.2, -0.3, 0.5]
                
        current_pose = kn.calculate_joint_angles(target_drop_location, position_type = "absolute")
        with eva.lock():
            eva.control_wait_for_ready()
            eva.control_go_to(current_pose)
                  
        # Drop object
        with eva.lock():
            eva.control_wait_for_ready()
            eva.gpio_set('ee_d0', False)
            eva.gpio_set('ee_d1', True)

        # 6. Complete the main program loop and return the arm to home position.
        print("\nProcess ended. Returning to home location.")
        
        target_position = [0, -0.2, 0.3];
        current_pose = kn.calculate_joint_angles(target_position, position_type = "absolute")
        with eva.lock():
            eva.control_wait_for_ready()
            eva.control_go_to(current_pose)
        
    # 7. Handle exceptions with “except KeyboardInterrupt:'' and “finally:” so that if problems occur the camera is shut down 
    #    and the arm stopped safely.  It is possible to use a “try” and “except” block to automatically reset the arm with 
    #    eva.control_reset_errors() if collisions occur, but this is rarely done in practice because for safety a human operator 
    #    should inspect the arm after an error before restarting automated motion.
    except KeyboardInterrupt:
       print("Exiting...")
       
    finally:
        # To automatically reset the arm. A human operator should inspect the arm after an error before restarting automated motion.
        # eva.control_reset_errors()
        pass;
    
if __name__ == "__main__":
    main();




