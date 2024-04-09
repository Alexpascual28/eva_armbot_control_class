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
import threading
import numpy as np
import csv
import os


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

        self.current_image = None # Instantiates variable to store current current image

        # Create CSV file with default hsv values if none exists already, and read those values to possible colours
        self.hsv_filename = 'hsv_colour_values.csv'
        self.current_hsv_values = self.read_hsv_values(self.hsv_filename)
        
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

    # Start continuous image acquisition
    def start_image_acquisition(self, time_period = 86400, show_feed = True):
        self.camera_thread = threading.Thread(target=self.show_camera_view, args=(time_period, show_feed))
        self.camera_thread.start()

    # Returns current image
    def get_current_image(self):
        return self.current_image
    
    # Displays image
    def show_image(self, frame_name, image):
        cv2.imshow(frame_name, image)

    # Stops image acquisition thread
    def stop_image_acquisition(self):
        self.camera_thread.join()

    # Shows continuous camera view
    def show_camera_view(self, time_period = 60, show_feed = True):
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
                    self.current_image = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)
            
                    # Show the image and wait a short time with:
                    if show_feed == True: cv2.imshow("capture", self.current_image)
                    cv2.waitKey(delay)
                    
        except KeyboardInterrupt:
           print("Exiting...")
           
        finally:
            # Stop acquisition and shut down the camera with:
            self.cam.stop_acquisition()
            self.cam.shutdown()
            print("Camera Off")

    # Takes one picture
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

    # Detect colour from given image
    def detect_colour(self, image, colour_name, frame_name="colour frame", image_format="hsv", show_frame = True):
            blurred_image = self.blurring(image)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            if colour_name in self.current_hsv_values:
                hsv_min = tuple(self.current_hsv_values.get(colour_name)[0])
                hsv_max = tuple(self.current_hsv_values.get(colour_name)[1])

                mask=cv2.inRange(hsv,hsv_min,hsv_max)
                masked_image = cv2.bitwise_and(image,image,mask=mask)

                self.frame_count += 1
                average_fps = self.frame_count / ( time.time() - self.start_time )
                cv2.putText(masked_image,"%2.1f fps" % average_fps, (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)

                # Show the frame
                if show_frame == True:
                    if frame_name == "colour frame": cv2.imshow(f"{colour_name} {frame_name}", masked_image)
                    else: cv2.imshow(f"{frame_name}", masked_image)

                return mask, masked_image
            else:
                print("Incorrect colour value.")
                return None

            # print(hsv_min)
            # print(hsv_max)

            # blueMin= (105,80,45)
            # blueMax= (155,255,230)

    # Detect shapes from given image 
    def detect_shapes(self, mask, shape_name, frame_name="Shape Frame", show_frame = False, return_largest = False):
        possible_shapes = {'triangle': 3, 'rectangle': 4, 'star': 10, 'circle': 11}

        if shape_name in possible_shapes:
            shapes = []

            mask = self.dilating(mask)
            mask = self.opening(mask)

            mask = self.eroding(mask)
            mask = self.closing(mask)
            
            mask = self.canny_edge_detection(mask)
            mask = self.dilating(mask)

            contours, h = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_SIMPLE)
            contours.sort(key = len)

            shape_counter = 0

            if return_largest == False:
                for contour in contours[-3:]:
                    #Amount of edges
                    approx = cv2.approxPolyDP(contour, 0.02*cv2.arcLength(contour, True), True)

                    #Center locations
                    M = cv2.moments(contour)
                    if M['m00'] == 0.0:
                        continue
                    centroid_x = int(M['m10']/M['m00'])
                    centroid_y = int(M['m01']/M['m00'])

                    if (len(approx) == possible_shapes.get(shape_name)) or (len(approx) >= 11 and shape_name == 'circle'):
                        shape = [f"{shape_name} {shape_counter}", contour, centroid_x, centroid_y, len(approx)]
                        shapes.append(shape)
                        shape_counter += 1

                return shapes
            
            else:
                # Select the largest one
                max_contour = max(contours, key = cv2.contourArea)

                #Amount of edges
                approx = cv2.approxPolyDP(max_contour, 0.02*cv2.arcLength(max_contour, True), True)

                #Center locations
                M = cv2.moments(max_contour)
                if M['m00'] != 0.0:
                    centroid_x = int(M['m10']/M['m00'])
                    centroid_y = int(M['m01']/M['m00'])

                    if (len(approx) == possible_shapes.get(shape_name)) or (len(approx) >= 11 and shape_name == 'circle'):
                        shape = [shape_name, max_contour, centroid_x, centroid_y, len(approx)]

                        return shape
                    
                    else:
                        print("No target shapes detected.")
                        return None
                else:
                    print("No contours detected.")
                    return None
                
        else:
            print("Incorrect shape value.")
            return None
    
    # Closing morphology
    def closing(self, mask):
        kernel = np.ones((7,7),np.uint8) 
        closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return closing

    # Opening morphology
    def opening(self, mask):
        kernel = np.ones((6,6),np.uint8)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return opening

    # Blurring morphology
    def blurring(self, mask):
        blur = cv2.GaussianBlur(mask,(5,5),0)
        return blur

    # Eroding morphology
    def eroding(self, mask):
        kernel = np.ones((5,5),np.uint8)
        erosion = cv2.erode(mask, kernel, iterations = 1)
        return erosion

    # Dilating morphology
    def dilating(self, mask):
        kernel = np.ones((5,5),np.uint8)
        dilation = cv2.dilate(mask, kernel, iterations = 1)
        return dilation

    # Canny edge detection morphology
    def canny_edge_detection(self, mask):
        edges = cv2.Canny(mask,100,200)
        return edges

    # Creates new csv file and populates it with default values if none exist with the same name
    def create_hsv_csv_file(self, filename):
        current_directory = os.getcwd()

        default_hsv_values = {"red": [(172,120,0), (180,255,255)], # red hsv range: low(179,0,0), high(180,255,255) // red,172,120,0,180,255,255
                            "blue": [(109,116,47),(115,255,100)], # blue hsv range: low(109,116,47), high(118,255,255) // blue,109,116,47,115,255,100
                            "green":[(65, 61, 0),(95, 255, 255)], # green hsv range: low(65, 61, 0), high(79, 255, 255) // green,65,61,0,95,255,255
                            "lilac":[(116, 60, 66),(152, 255, 255)],} # lilac hsv range: low(116, 60, 66), high(152, 255, 255) // lilac,116,60,66,152,255,255

        if not os.path.isfile(current_directory + '/' + filename):
            self.write_hsv_values(filename, default_hsv_values)

    # Read hsv values from file
    def read_hsv_values(self, filename):
        self.create_hsv_csv_file(filename) # Create new file if it does not exist

        with open(filename, mode='r') as file:
            reader = csv.reader(file)
            hsv_values = {rows[0]:[[int(rows[1]), int(rows[2]), int(rows[3])], [int(rows[4]), int(rows[5]), int(rows[6])]] for rows in reader}

        return hsv_values
    
    # Write hsv values  to file
    def write_hsv_values(self, filename, hsv_values):
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            for colour in hsv_values:
                writer.writerow([colour, hsv_values[colour][0][0], hsv_values[colour][0][1], hsv_values[colour][0][2],
                                         hsv_values[colour][1][0], hsv_values[colour][1][1], hsv_values[colour][1][2]])
    
    # Change hsv values from file
    def change_hsv_values(self, colour, parameter, range, value):
        self.current_hsv_values = self.read_hsv_values(self.hsv_filename)

        colours = ["red", "blue", "green", "lilac"]
        parameters = ["hue", "saturation", "value"]
        ranges = ["low", "high"]

        if colour in colours and parameter in parameters and range in ranges:
            self.current_hsv_values[colour][ranges.index(range)][parameters.index(parameter)] = value
            self.write_hsv_values(self.hsv_filename, self.current_hsv_values)
        else:
            print("Invalid input")
