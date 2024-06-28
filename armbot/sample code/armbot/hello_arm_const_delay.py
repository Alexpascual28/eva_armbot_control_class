# -*- coding: utf-8 -*-
"""
Created on Mon Nov 13 17:23:18 2023

@title: Armbot class test 1
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

import armbot as ab
import time

def main():
    # Connect to the arms
    # https://wiki.york.ac.uk/display/TSS/Network+Details+and+Credentials+for+the+EVA+Arms+and+Network+Cameras
    arm = ab.ArmBot("evatrendylimashifterpt410");
    
    # Start block
    try:     
        arm.home_robot();
        
        pose1 = arm.move_end_efector([0.2, -0.2, 0.5]);
        arm.open_gripper();
        time.sleep(0.1)
        arm.close_gripper();
        time.sleep(0.1)
        arm.open_gripper();
        time.sleep(0.1)
        arm.close_gripper();
        
        pose2 = arm.move_end_efector([-0.2, -0.2, 0.5]);
        arm.open_gripper();
        time.sleep(0.1)
        arm.close_gripper();
        time.sleep(0.1)
        arm.open_gripper();
        time.sleep(0.1)
        arm.close_gripper();
        
        arm.set_joint_angles(pose1)
        arm.open_gripper();
        time.sleep(0.1)
        arm.close_gripper();
        time.sleep(0.1)
        arm.open_gripper();
        time.sleep(0.1)
        arm.close_gripper();
        
        arm.set_joint_angles(pose2)
        arm.open_gripper();
        time.sleep(0.1)
        arm.close_gripper();
        time.sleep(0.1)
        arm.open_gripper();
        time.sleep(0.1)
        arm.close_gripper();
        
        arm.home_robot();
        
    except KeyboardInterrupt:
       print("Exiting...")
       
    finally:
        # To automatically reset the arm. A human operator should inspect the arm after an error before restarting automated motion.
        # eva.control_reset_errors()
        pass;
    
if __name__ == "__main__":
    main();




