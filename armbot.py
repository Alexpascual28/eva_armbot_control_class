# -*- coding: utf-8 -*-
"""
Created on Mon Nov  17:23:18 2023

@title: ArmBot Class
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

from kinematics import Kinematics
from evasdk import Eva
import time
import cv2
from aravis import Camera

# 2. Initialize your kinematics functions and variables including the constants (dimensions, etc.) for the EVA robot arm.
class ArmBot:
    def __init__(self, arm_hostname):
        print("\n---------------------------------------------------------------------\n");
        self.kn = self.initialize_kinematics();
        self.eva = self.initialize_arm(arm_hostname);
        self.cam = self.initialize_camera(arm_hostname);
        
        self.home_pose = None; # Initializes home_pose as None as the pose is updated the first time the robot is homed.
        
        # 4. Move the arm to a starting position, ideally near where your target is located so that your camera can see it, 
        #    and initialize the state of the robot arm to have no object in its gripper and to move to pick up.
        if self.home_pose is None:
            print("\nSetting Home position for robot arm...")
            home_position = [0, -0.2, 0.3];
            self.home_pose = self.kn.calculate_joint_angles(home_position, position_type = "absolute")
            print("\nDone.\n")

        print("\nArm \"" + arm_hostname + "\" is ready to use!\n");
        print("\n---------------------------------------------------------------------\n");
        
    # Initializes main kinematics class to assist with robot movement
    def initialize_kinematics(self):
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
    def initialize_arm(self, arm_hostname):
        print("\nInitialising EVA robot arm...")
        
        # Details to connect to the robot arm, more info here: 
        # https://wiki.york.ac.uk/display/TSS/Network+Details+and+Credentials+for+the+EVA+Arms+and+Network+Cameras 
        arm1_hostname = "evatrendylimashifterpt410" 
        arm1_ip = "144.32.152.105"
        token1 = "1462980d67d58cb7eaabe8790d866609eb97fd2c"

        arm2_hostname = "evacunningyorkartistpt410" 
        arm2_ip = "144.32.152.2"
        token2 = "4b1c26278a566e0419165a3acd025dd83d32b160" 
      
        # Create an “eva” object with these parameters to connect to the arm itself on the network
        if arm_hostname == arm1_hostname:
            eva = Eva(arm1_ip, token1)
        else:
            eva = Eva(arm2_ip, token2)
            
        print("Done.\n")
        return eva

    # Initializes camera module by connecting using the specified details
    def initialize_camera(self, arm_hostname):
        print("\nInitialising camera...")
        
        # Conexion details for the camera
        camera1_hostname = "evacctv02"
        camera1_ip = "144.32.152.10"
        camera1_id = 'S1188411'
        arm1_hostname = "evatrendylimashifterpt410" 

        camera2_hostname = "evacctv03"
        camera2_ip = "144.32.152.11"
        camera2_id = 'S1188413'
        arm2_hostname = "evacunningyorkartistpt410"

        # Create a “cam” object with these parameters to connect to the camera itself on the network based on the arm name
        if arm_hostname == arm1_hostname:
            cam = Camera(camera1_id)
        else:
            cam = Camera(camera2_id)
        
        # You can initialize a Camera object and set its parameters with:
        cam.set_feature("Width", 1936)
        cam.set_feature("Height", 1216)
        cam.set_frame_rate(10)
        cam.set_exposure_time(100000)
        cam.set_pixel_format_from_string('BayerRG8')

        # Print out the camera parameters in use
        print("\nCamera model: ", cam.get_model_name())
        print("Vendor Name: ", cam.get_vendor_name())
        print("Device id: ", cam.get_device_id())
        print("Region: ", cam.get_region(), end="")
        print("\n")

        print("Done.\n")
        
        return cam

    # Moves arm to home position ([0, -0.2, 0.3])
    def home_robot(self):
        with self.eva.lock():
            self.eva.control_wait_for_ready()
            self.eva.control_go_to(self.home_pose)

    # Moves to an absolute coordinate address (inverse kinematics)
    def move_end_efector(self, absolute_position):
        if absolute_position[0] <= -0.3:
            absolute_position[0] = -0.3
        elif absolute_position[0] >= 0.3:
            absolute_position[0] = 0.3

        if absolute_position[1] <= -0.5:
            absolute_position[1] = -0.5
        elif absolute_position[1] >= -0.1:
            absolute_position[1] = -0.1

        if absolute_position[2] <= 0.18:
            absolute_position[2] = 0.18
        elif absolute_position[2] >= 0.5:
            absolute_position[2] = 0.5
        
        current_pose = self.kn.calculate_joint_angles(absolute_position, position_type = "absolute")
        with self.eva.lock():
            self.eva.control_wait_for_ready()
            self.eva.control_go_to(current_pose)
            
        return current_pose

    # Shifts end efector from the current position (inverse kinematics)
    def shift_end_efector(self, relative_position):
        current_pose = self.kn.calculate_joint_angles(relative_position, position_type = "relative")
        with self.eva.lock():
            self.eva.control_wait_for_ready()
            self.eva.control_go_to(current_pose)
            
        return current_pose
    
    # Moves arm by defining joint angles (forward kinematics)
    def set_joint_angles(self, pose):
        with self.eva.lock():
            self.eva.control_wait_for_ready()
            self.eva.control_go_to(pose)

    # Opens gripper
    def open_gripper(self):
        # Open the gripper (to get rid of whatever is being gripped and test the gripper)
        with self.eva.lock():
            self.eva.control_wait_for_ready()
            self.eva.gpio_set('ee_d0', True)
            self.eva.gpio_set('ee_d1', False)

    # Closes gripper
    def close_gripper(self):
        # Close your gripper (or open it, depending on the design you have made), 
        # you need to set ee_d1 to a low logic state and ee_d2 to a high logic state
        with self.eva.lock():
            self.eva.control_wait_for_ready()
            self.eva.gpio_set('ee_d1', True)
            self.eva.gpio_set('ee_d0', False)

    # Shows continuous camera view
    def show_camera_view(self, time_period):
        try:
            # Start the camera
            self.cam.start_acquisition_continuous()
            print("Camera On")
            
            # Open an OpenCV window to view the image
            cv2.namedWindow('capture', flags=0)
            
            i = 0
            delay = 20 # Delay in miliseconds per iteration
            maximum_i = (time_period * 1000) / delay;
            
            while i < maximum_i:
                i += 1
                
                # Capture an individual frame
                frame = self.cam.pop_frame()
                print("[", time.time(), "] frame nb: ", i, " shape: ", frame.shape)
                
                if not 0 in frame.shape:
                    # Convert to standard RGB format
                    image = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)
            
                    # Show the image and wait a short time with:
                    cv2.imshow("capture", image)
                    cv2.waitKey(delay)
                    
        except KeyboardInterrupt:
           print("Exiting...")
           
        finally:
            # Stop acquisition and shut down the camera with:
            self.cam.stop_acquisition()
            self.cam.shutdown()
            print("Camera Off")

    def take_picture(self):
        try:
            # Start the camera
            self.cam.start_acquisition_continuous()
            print("Camera On")

            # Capture an individual frame
            frame = self.cam.pop_frame()
            print("[", time.time(), "]", " shape: ", frame.shape)
                
            if not 0 in frame.shape:
                # Convert to standard RGB format
                image = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)

                return image
                 
        except KeyboardInterrupt:
           print("Exiting...")
           
        finally:
            # Stop acquisition and shut down the camera with:
            self.cam.stop_acquisition()
            self.cam.shutdown()
            print("Camera Off")

