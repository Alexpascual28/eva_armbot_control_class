# -*- coding: utf-8 -*-
"""
Created on Mon May  1 13:55:01 2023

@title: Kinematics class
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York

"""

# 1. import all the modules/libraries for Python that are needed for your code, including NumPy, SymPy, MatPlotLib 
#    (if you want to use plotting of your kinematics as before), the evasdk module, and the aravis module.
import numpy
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset, solve, simplify, S, diff, det, erf, log, sqrt, pi, sin, cos, tan, atan2, init_printing
   
# 2. Initialize your kinematics functions and variables including the constants (dimensions, etc.) for the EVA robot arm.
class Kinematics:
    def __init__(self, arm_dimensions, initial_pose, simulate, dp_threshold, step_size,
                 theta_max_step, pause, plot_dimensions, camera_view):
        print("\nStarting kinematics module...")
        
        self.arm_dimensions = arm_dimensions # Dimensions of each link of the robot in metres e.g: [0.187, 0.096, 0.205, 0.124, 0.167, 0.104]
        self.initial_pose = initial_pose # Robot initial pose (in joint angles) e.g: [0, 1, -2.5, 0, -1.6, 0]
        self.simulate = simulate # Whether the calculations should be displayed as a graphical simulation e.g: True
        
        # Variables to control calculation speed and accuracy
        self.dp_threshold = dp_threshold # Calculations will stop when dp is lower than the threshold
        self.step_size = step_size # In each step, the robot will move this amount
        self.theta_max_step = theta_max_step # Each joint won't rotate more than this angle in radians
        self.pause = pause # Time the plot will be displayed
        
        # Variables to control how the plot is displayed
        self.plot_dimensions = plot_dimensions # Must be a 3 x 2 matrix specifying min and max limits for every axis
        self.camera_view = camera_view # must be a 1 x 2 array specifying elev and azim
        
        self.elements = self.define_initial_elements();
        self.current_pose = self.initial_pose
    
    # Define dimensions and configuration of the robot arm, as well as symbols and 
    # initial values of variables used for kinematics
    def define_initial_elements(self):
        print("Defining robot dimensions and initial position.");
        
        # Define your joint angles as SymPy symbols
        theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')
        theta = Matrix([theta1,theta2,theta3,theta4,theta5,theta6])
        
        # Define transforms to each joint
        T1 = self.Ry(-pi/2) * self.T(self.arm_dimensions[0], 0, 0) * self.Rx(theta1)
        T2 = T1 * self.T(self.arm_dimensions[1], 0, 0) * self.Rz(theta2)
        T3 = T2 * self.T(self.arm_dimensions[2], 0, 0) * self.Rz(theta3)
        T4 = T3 * self.T(self.arm_dimensions[3], 0, 0) * self.Rx(theta4)
        T5 = T4 * self.T(self.arm_dimensions[4], 0, 0) * self.Rz(theta5)
        T6 = T5 * self.T(self.arm_dimensions[5], 0, 0) * self.Rx(theta6)
        
        # Find joint positions in space
        # p0, p1, p2, and p3 are points in 3D space defined as symbolic Matrices that 
        # can be transformed mathematically – note that they are actually four-dimensional 
        # so they can be used with homogeneous transforms with the last ‘W’ coordinate 
        # always 1 (to facilitate translation as well as rotation).
        p0 = Matrix([0,0,0,1])
        p1 = T1 * p0
        p2 = T2 * p0
        p3 = T3 * p0
        p4 = T4 * p0
        p5 = T5 * p0
        p6 = T6 * p0
        
        p = Matrix([p6[0], p6[1], p6[2]]) # coordinates of arm tip
        
        # Initial position θi to start movement from (straight up position, all joint angles=0)
        # theta_i = Matrix([0,0,0,0,0,0])
        # Or you could use the “home” position from lab 3, 
        # though you need to change the variable names to use for substitution:
        # theta_i = Matrix([-pi/2,-pi/4,3*pi/4,0,pi/2,0])
        theta_i = Matrix(self.initial_pose);
        
        # Calculate the initial (current) point of the end effector directly 
        # from the joint angles by substituting theta in the forward kinematics of point p
        p_i = p.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2],
                      theta4:theta_i[3], theta5:theta_i[4],theta6:theta_i[5]}).evalf()
        
        # Define the initial final point and the point differential
        p_f = Matrix([0, 0, 0]);
        dp = p_f - p_i;
        
        print("Done.\n")
        
        return [theta1, theta2, theta3, theta4, theta5, theta6, theta, 
                theta_i, p0, p1, p2, p3, p4, p5, p6, p, p_i, p_f, dp]

    def calculate_joint_angles(self, final_position, position_type):
        print("\nCalculating robot pose for " + position_type + " position: ",
              final_position, " from current pose: ", self.current_pose, end="")
        
        # Current position θi to start movement from 
        theta_i = Matrix(self.current_pose);
        
        theta1, theta2, theta3, theta4, theta5, theta6 = self.elements[0:6]
        
        # Calculate the initial (current) point of the end effector directly 
        # from the joint angles by substituting theta in the forward kinematics of point p
        p = self.elements[-4]
        p_i = p.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2],
                      theta4:theta_i[3], theta5:theta_i[4],theta6:theta_i[5]}).evalf()
        
        # Depending on the position type, set a relative or an absolute position for the target end efector
        if position_type == "relative":
            # Define the final (target) point of the end effector also as simply 
            # a relative movement from your initial position, which you can do 
            # as follows for the example of moving the arm down in the z axis by 1cm
            p_f = p_i + Matrix(final_position);
        else:
            # Or define a final point as set coordinates (absolute position)
            p_f = Matrix(final_position);
        
        # You can model the movement of the end of the arm with the 
        # difference dp = pf-pi and this must be caused by the 
        # joint angles θ moving by a difference dθ
        dp = p_f - p_i;
        
        self.elements[-1] = dp
        self.elements[-2] = p_f
        self.elements[-3] = p_i
        
        # dp.threshold value to exit the loop
        # step_size and theta_step used to move in small increments, to reduce the vector dp into incremental steps.
        # Example: Step the end effector position by 0.5mm each iteration
        # and change angle by no more than 0.2 radians
        
        # Loop to iterate through a number of points from the start position pi to the end position pf
        while dp.norm() > self.dp_threshold:
            
            self.elements, X, Y, Z, W = self.calculate_next_position()
            dp = self.elements[-1]
            theta_i = self.elements[7]
            p_i = self.elements[-3]
            
            if(self.simulate == True):
                # Draw current position in plot
                ax = self.create_plot();
                self.draw_elements_in_plot(ax, X, Y, Z);

                # Print current step position and rotation
                # print("\nstep “,step,”:\n θ[",theta_i,"]\n p[",p_i,"]")
                
            # Print dot to represent an iteration
            print(".", end = "")
        
        theta_i = numpy.array(theta_i.evalf()).tolist(); #.astype(numpy.float64);
        theta_i = [float(theta_i[0][0]), float(theta_i[1][0]), float(theta_i[2][0]), float(theta_i[3][0]), float(theta_i[4][0]), float(theta_i[5][0])]
        
        # Print final joint angles
        print('\n\nFinal Joint Angles in Radians:\n', theta_i)
        self.current_pose = theta_i;

        return theta_i;

    def calculate_next_position(self):
        # Import variables from elements
        [theta1, theta2, theta3, theta4, theta5, theta6, theta, theta_i,
         p0, p1, p2, p3, p4, p5, p6, p, p_i, p_f, dp] = self.elements;
        
        # dp represents the distance between where the end effector is now and
        # our goal position. We can define a step size for x,y, and z coordinates
        # dp_step as follows
        dp_step = dp * self.step_size / dp.norm()
        
        # To calculate the numerical value of J (jacobian) at each point,
        # you can produce the Jacobian as follows
        J = p.jacobian(theta)
        J_i = J.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2],
                      theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        
        # Solve the expression to find values of θ for the new point (solve for dθ)
        # To do this we calculate the inverse of J, and multiply this by dp_step, since J*dθ=dp => dθ=dp*J^-1
        J_inv = J_i.pinv()
        dtheta = J_inv * dp_step
        
        # Update your theta_i joint values, limiting the joint movement to theta_max_step, 
        # which can be done with numpy.clip()
        theta_i = theta_i + numpy.clip(dtheta,-1*self.theta_max_step,self.theta_max_step)
        
        # Update the new p_i from your forward kinematics
        p_i = p.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2],
                      theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        
        # Update dp to reflect the current vector to the goal position
        dp = p_f - p_i

        # Plot the actual position of the arm in each loop
        p0sub = p0.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        p1sub = p1.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        p2sub = p2.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        p3sub = p3.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        p4sub = p4.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        p5sub = p5.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        p6sub = p6.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], 
                         theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
        
        # numpy.array() concatenates these points into a column (enter “soa” to see)
        soa = numpy.array([p0sub,p1sub,p2sub,p3sub,p4sub,p5sub,p6sub])
        
        # zip() extracts the columns from soa so that they are column vectors of X, Y,
        # and Z components, with the W component unused for 3D projection (always=1)
        X, Y, Z, W = zip(*soa)
        
        # additional calls to numpy.array() and then numpy.ndarray.flatten() are to 
        # ensure that a flat set of vectors result
        X = numpy.array(X)
        Y = numpy.array(Y)
        Z = numpy.array(Z)
        W = numpy.array(W)
        
        X = numpy.ndarray.flatten(X)
        Y = numpy.ndarray.flatten(Y)
        Z = numpy.ndarray.flatten(Z)
        W = numpy.ndarray.flatten(W)
        
        # Variables are exported within the elements array
        elements = [theta1, theta2, theta3, theta4, theta5, theta6, theta, theta_i,
        p0, p1, p2, p3, p4, p5, p6, p, p_i, p_f, dp]
        
        return elements, X, Y, Z, W

    def create_plot(self):
        # fig.add_subplot(111, projection='3d') creates MatPlotLib axes that you 
        # can plot vectors and points on to
        fig = matplotlib.pyplot.figure(1)
        ax = fig.add_subplot(111, projection='3d')

        # set_xlabel() sets the label to a string argument on a parent axis
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        
        ax.set_xlim([self.plot_dimensions[0][0], self.plot_dimensions[0][1]])
        ax.set_ylim([self.plot_dimensions[1][0], self.plot_dimensions[1][1]])
        ax.set_zlim([self.plot_dimensions[2][0], self.plot_dimensions[2][1]])
        
        # Change initial orientation of the plot view perspective
        ax.view_init(elev=self.camera_view[0], azim=self.camera_view[1])
        
        return ax;

    def draw_elements_in_plot(self, ax, X, Y, Z):
        # ax.plot3D(X,Y,Z, 'blue', marker="o") plots lines connecting the points specified
        # in X, Y, and Z in the colour green (you can change this colour as you wish) and 
        # with the marker style specified (we use “o” for circles at joints)
        ax.plot3D(X,Y,Z, 'blue', marker="o")

        # matplotlib.pyplot.draw() draws the plot, matplotlib.pyplot.show() shows the 
        # plot, and matplotlib.pyplot.pause() runs the plotting event loop for a time.
        matplotlib.pyplot.draw()
        matplotlib.pyplot.show()
        matplotlib.pyplot.pause(self.pause)
        
        return;

    # Translation homogeneous matrix
    def T(self, x, y, z):
       T_xyz = Matrix([[1,         0,          0,          x],
                       [0,         1,          0,          y],
                       [0,         0,          1,          z],
                       [0,         0,          0,          1]])
       return T_xyz

    # Rotation along x axis homogeneous matrix
    def Rx(self, roll):
       R_x = Matrix([[1,         0,          0, 0],
                     [0, cos(roll), -sin(roll), 0],
                     [0, sin(roll),  cos(roll), 0],
                     [0,         0,          0, 1]])
       return R_x

    # Rotation along y axis homogeneous matrix
    def Ry(self, pitch):
       R_y = Matrix([[ cos(pitch), 0, sin(pitch), 0],
                     [          0, 1,          0, 0],
                     [-sin(pitch), 0, cos(pitch), 0],
                     [          0, 0,          0, 1]])
       return R_y

    # Rotation along z axis homogeneous matrix
    def Rz(self, yaw):
       R_z = Matrix([[cos(yaw),-sin(yaw), 0, 0],
                     [sin(yaw), cos(yaw), 0, 0],
                     [       0,        0, 1, 0],
                     [       0,        0, 0, 1]])
       return R_z

    # Compound rotation homogeneous matrix. 
    # The order of rotations must be consistent.
    def R(self, roll, pitch, yaw):
       R_x = Matrix([[1,         0,          0],
                     [0, cos(roll), -sin(roll)],
                     [0, sin(roll),  cos(roll)]])

       R_y = Matrix([[ cos(pitch), 0, sin(pitch)],
                     [          0, 1,          0],
                     [-sin(pitch), 0, cos(pitch)]])

       R_z = Matrix([[cos(yaw),-sin(yaw), 0],
                     [sin(yaw), cos(yaw), 0],
                     [       0,        0, 1]])
       return R_z*R_y*R_x
