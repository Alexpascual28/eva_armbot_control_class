# -*- coding: utf-8 -*-
"""
Created on Mon Nov 13 17:23:18 2023

@title: Armbot class test 1
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

# 1. import all the modules/libraries for Python that are needed for your code, including NumPy, SymPy, MatPlotLib 
#    (if you want to use plotting of your kinematics as before), the evasdk module, and the aravis module.
from armbot import ArmBot
import time

def main():
    # Connect to the arms
    # Details to connect to the robot arm, more info here: 
    # https://wiki.york.ac.uk/display/TSS/Network+Details+and+Credentials+for+the+EVA+Arms+and+Network+Cameras 
    arm = ArmBot("evatrendylimashifterpt410");
    
    # Start block
    try:     
        arm.home_robot();

        arm.open_gripper();

        arm.move_end_efector([0, -0.2, 0.18]);

        arm.close_gripper();

        arm.home_robot();

        arm.move_end_efector([0.2, -0.2, 0.22]);

        arm.open_gripper();
        
        arm.home_robot();

        arm.close_gripper();
        
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



