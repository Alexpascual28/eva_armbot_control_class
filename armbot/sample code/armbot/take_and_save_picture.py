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
import cv2

def main():
    # Connect to the arms
    # Details to connect to the robot arm, more info here: 
    # https://wiki.york.ac.uk/display/TSS/Network+Details+and+Credentials+for+the+EVA+Arms+and+Network+Cameras
    arm1 = ArmBot("evatrendylimashifterpt410");
    
    # Start block
    try:     
        arm1.home_robot();
        arm1.open_gripper();
        arm1.move_end_efector([0, -0.2, 0.18]);

        # arm1.show_camera_view(2); # Show for two seconds
        
        # time.sleep(2)

        # Save the image to a picture file
        image = arm1.take_picture();
        path = "image.png"
        print("Saving image to", path)
        cv2.imwrite(path, image)
        
    # 7. Handle exceptions
    except KeyboardInterrupt:
       print("Exiting...")
       
    finally:
        # To automatically reset the arm. A human operator should inspect the arm after an error before restarting automated motion.
        # arm1.eva.control_reset_errors();
        pass;
    
if __name__ == "__main__":
    main();



