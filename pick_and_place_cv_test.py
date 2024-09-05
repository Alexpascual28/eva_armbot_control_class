# -*- coding: utf-8 -*-
"""
Created on Mon May  1 17:23:18 2023

@title: Implement Picking-Up Motion
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

# 1. import all the modules/libraries for Python that are needed for your code
import numpy as np
import armbot as ab
import cv2
import time

def main():
    try:
        # Connect to the arm
        arm = ab.ArmBot("evatrendylimashifterpt410"); # Armbot names = evacunningyorkartistpt410 / evatrendylimashifterpt410

        # Home robot
        arm.home_robot();
        
        # Start camera acquisition & open an OpenCV window to view the image
        arm.start_image_acquisition(show_feed=True);

        while(True):
            # Capture an individual image
            image = arm.get_current_image()

            if image is not None:
                # Show the image and wait a short time with:
                arm.show_image("Capture", image)
                print("[", time.time(), "] initial frame. ""Shape: ", image.shape)
                    
                # 4. Move the arm to a starting position, ideally near where your target is located so that your camera can see it, 
                #    and initialize the state of the robot arm to have no object in its gripper and to move to pick up.
                initial_position = [0, -0.15, 0.3];
                initial_pose = arm.move_end_efector(initial_position);
                
                # Open the gripper (to get rid of whatever is being gripped and test the gripper)
                arm.open_gripper();
                time.sleep(0.5)
                
                # Close your gripper (or open it, depending on the design you have made), 
                # you need to set ee_d1 to a low logic state and ee_d2 to a high logic state
                arm.close_gripper();
                
                # 5. Start your main program loop, in which you will probably need to:
                t = 0
                t_timeout = 50000 
                
                # Start main loop
                while t < t_timeout:
                    i = 0
                    maximum_i = 1000
                    
                    is_gripper_over_object = False
                    is_aligned_x = False;
                    is_aligned_y = False;
                    is_aligned_z = False;
                    
                    # Start loop to detect if gripper is directly over the target
                    while i < maximum_i and is_gripper_over_object == False:
                        i += 1
                    
                        # a. Capture a frame from the camera in OpenCV format.
                        image = arm.get_current_image()
                        
                        # If a frame is taken (frame.shape is not empty)
                        if image is not None:
                            
                            # b. Perform shape detection or blob detection using OpenCV to identify the target object to pick up.
                            # Detect blue colour
                            blue_mask, _ = arm.detect_colour(image, "red", show_frame=False)
                            
                            # Detect largest rectangle within image
                            shapes, _ = arm.detect_shapes(blue_mask, "rectangle", show_frame=False)

                            if len(shapes) is not 0:
                                shape = shapes[0]
                            else:
                                shape = None

                            if shape is not None:
                                # Calculate minimum rectangle area parameters
                                rectangle = cv2.minAreaRect(shape[1])
                                [[center_x, center_y], [contour_width, contour_height], angle_of_rotation] = rectangle

                                # Get corner points of the rectangle
                                box = cv2.boxPoints(rectangle)
                                box = np.intp(box)

                                # c. Show the frame in the OpenCV window with detection annotations.
                                # cv2.putText(image, f"Rectangle {shape[0]}", (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                                # cv2.drawContours(image, [box], -1, (0, 255, 0), 2)
                                arm.draw_shape(image, shape)
                    
                                # Show the image and wait a short time with:
                                # cv2.imshow("Capture 2", image)
                                # cv2.waitKey(1)
                                arm.show_image("Capture 2", image)
                                
                                # d. Identify the location of the target object or the target drop location and convert this target location
                                #    into the coordinate system that you are using for kinematics.
                                
                                image_height, image_width, image_channels = image.shape # Image dimesions (width, height) in pixels
                                
                                pixel_size = 0.00000586 # Pixel size in metres (4.8 micrometers)
                                sensor_width_m = pixel_size * image_width # Width of the camera sensor in metres
                                focal_length_m = 0.016 # Camera focal length in metres (16 milimetres).
                                
                                focal_length_p = focal_length_m * image_width / sensor_width_m # Rough estimate of focal length in px. Must be in pixels! Can be calibrated to find out more accurately.
                                known_width = 0.02 # Known width of object in metres (2 centimetres).
                                
                                # Calculate position of center point of contour in 3D space (from the camera)
                                center_z = (known_width * focal_length_p) / contour_width # Distance to the object in metres
                                # point_xyz = [center_x, center_y, center_z] # point is in Camera Optical frame, i.e. point[0] x-right (px),  point[1] y-down (px), point[2] z-forward (m)
            
                                # e. Calculate inverse kinematics for the robot arm to move to the next step in direction towards the target 
                                #    location - it is usually better to move as a series of small steps rather than a single large motion 
                                #    but it depends on how your inverse kinematics are implemented.
                                
                                # Robot movement step size and target distance
                                robot_step_size = 0.01 # in metres (e.g 1 cm -> 0.01 m)
                                target_distance_to_object = 0.15 # Target distance to the object (e.g 15 cm -> 0.15 m)
                                margin = 0.01 # Margin of alignment to the object
                                
                                # Adjust x axis
                                if (center_x > (image_width / 2) + margin): # If the object is too much to the left
                                    target_position_x = robot_step_size; # Move to the right
                                    is_aligned_x = False;
                                    
                                elif (center_x < (image_width / 2) - margin): # If the object is too much to the right
                                    target_position_x = -robot_step_size; # Move to the left
                                    is_aligned_x = False;
                                    
                                else: # If the object is in the centre (within the margin)
                                    is_aligned_x = True;
                                    target_position_x = 0; # Stop
                                    
                                # Adjust y axis
                                if (center_y > (image_height / 2) + margin): # If the object is too up in the image
                                    target_position_y = robot_step_size; # Move down
                                    is_aligned_y = False;
                                    
                                elif (center_y < (image_height / 2) - margin): # If the object is too down in the image
                                    target_position_y = -robot_step_size; # Move up
                                    is_aligned_y = False;
                                    
                                else: # If the object is in the centre (within the margin)
                                    is_aligned_y = True;
                                    target_position_y = 0; # Stop
                                    
                                # Adjust z axis
                                if (center_z > target_distance_to_object + margin): # If the object is too far away
                                    target_position_z = -robot_step_size; # Move forward
                                    is_aligned_z = False;
                                    
                                elif (center_z < target_distance_to_object - margin): # If the object is too near
                                    target_position_z = robot_step_size; # Move backwards
                                    is_aligned_z = False;
                                    
                                else: # If the object at the correct distance (within the margin)
                                    is_aligned_z = True;
                                    target_position_z = 0; # Stop
                                    
                                # Add all movements together (along the three axis)
                                target_position = [target_position_x, target_position_y, target_position_z];
                
                                # Move the robot
                                arm.shift_end_efector(target_position);
                                
                                # If the gripper is aligned right on top of the object (on the three axis), break the loop (see beginning of while statement)
                                is_gripper_over_object = is_aligned_x and is_aligned_y and is_aligned_z;
                                
                    # f. Detect when the gripper on the robot arm is located around the target object, and close the gripper, 
                    #    or else detect that the arm has reached the drop location and open the gripper.
                    if is_gripper_over_object:
                        # Open gripper
                        arm.open_gripper();
                        
                        # Move towards the object
                        pick_up_position = [0, 0, -0.05] # Object pick up distance, relative to the camera
                        arm.shift_end_efector(pick_up_position);
                        
                        # Close gripper
                        arm.close_gripper();
                        
                        # Move up
                        pick_up_position = [0, 0, 0.3] # Object pick up distance, relative to the camera
                        arm.shift_end_efector(pick_up_position);
                        
                        # Move to target drop location
                        target_drop_location = [0.2, 0.3, 0.1]
                        arm.move_end_efector(target_drop_location)
                        
                        # Drop object
                        arm.open_gripper()

                        # Move to initial position
                        arm.set_joint_angles(initial_pose)
                
                        # g. Repeat and change the current status of the arm if it has completed picking up or dropping an object.
                        is_gripper_over_object = False
                        
                    else:
                        print("\nTimeout. No object was detected, or the robot arm failed to align itself with the object. Retrying.\n")

                # 6. Complete the main program loop and return the arm to home position.
                print("\nProcess ended. Returning to home location.")
                arm.home_robot();
                
    except KeyboardInterrupt:
        print('Interrupted!')
        
if __name__ == "__main__":
    try:
        main();

    # 7. Handle exceptions with “except KeyboardInterrupt:'' and “finally:” so that if problems occur the camera is shut down 
    #    and the arm stopped safely.  It is possible to use a “try” and “except” block to automatically reset the arm with 
    #    eva.control_reset_errors() if collisions occur, but this is rarely done in practice because for safety a human operator 
    #    should inspect the arm after an error before restarting automated motion.
    except KeyboardInterrupt:
        print("Exiting...")
