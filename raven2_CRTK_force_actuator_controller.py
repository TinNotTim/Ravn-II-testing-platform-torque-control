"""
Raven 2 Control - Control software for the Raven II robot
Copyright (C) 2005-2022  Andrew Lewis, Yun-Hsuan Su, Haonan Peng, Blake Hannaford,
and the University of Washington BioRobotics Laboratory
This file is part of Raven 2 Control.
Raven 2 Control is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
Raven 2 Control is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
raven_py_controller.py
Python controller for RAVEN II using CRTK API
date Apr 11, 2023
author Haonan Peng, Dun-Tin Chiang, Yun-Hsuan Su, Andrew Lewis

[!!!IMPORTANT!!!]: This is a hacking code to use the RAVEN II right arm control board to control RAVEN's motor providing desired torque, but not controlling the actual right arm
[!!!IMPORTANT!!!]: This code hack into CRTK 'servo_jp' control and replace its function into torque control
[!!!IMPORTANT!!!]: The usage of this code requires change in RAVEN's src code, if you would like to use this code, please contact authors at penghn@uw.edu
"""

# This force actuator controller use raven2 CRTK torque control to provide a desired force to RAVEN's left arm end-effector

testing = False

import time


import numpy as np
import math
from scipy.optimize import minimize
# from scipy.spatial.transform import Rotation as sp_rot

if not testing:
    import rospy
    import std_msgs.msg
    import geometry_msgs.msg
    import sensor_msgs.msg

    import crtk_msgs.msg # crtk_msgs/operating_state
    #todo import the msgs.type of raven state
    from raven_2.msg import raven_state
    import raven2_CRTK_torque_controller


class raven2_crtk_force_controller():

    # ros node is not initialized here, it should be initialized and named by the program that use this controller
    def __init__(self):

        self.motor_loc = np.array([[1000, 1000, 1000],     # (x, y, z) motor locations in mm, in RAVEN's frame 0, transform might be needed, [0] not used, [1] for motor 1 
                                  [500, 0, 0],
                                  [-500, 0, 0],
                                  [0, 500, 0],
                                  [0, -500, 0],
                                  [0, 0, 500],
                                  [0, 0, -500]
                                  ])

        

        self.motor_dir = np.array([[1, 0, 0],     # unit vector of the direction of each motor, computed by motor location and end-effector location, transform might be needed, [0] not used, [1] for motor 1 
                                  [1, 0, 0],
                                  [-1, 0, 0],
                                  [0, 1, 0],
                                  [0, -1, 0],
                                  [0, 0, 1],
                                  [0, 0, -1]
                                  ])
        
        self.loadcell_dir = np.array([[1, 0, 0],     # unit vector of the direction of each load cell, for calculating the angle of cable and load cell, fixed value, [0] not used, [1] for motor 1 
                                  [1, 0, 0],
                                  [-1, 0, 0],
                                  [0, 1, 0],
                                  [0, -1, 0],
                                  [0, 0, 1],
                                  [0, 0, -1]
                                  ])


        #variable for end_effector location
        self.end_effector_loc = None 
        # self.end_effector_loc = np.array([-77.0, -25.0, 14.0])

        #variable for angle between cables and load cell direction
        self.cable_load_cell_angles = np.zeros(7) # unit: rad, will be updated by compute_cable_angles() function, [0] not used, [1] for motor 1 

        #variable for actual load cell force reading
        self.__callback_load_cell_force = np.zeros(7) # unit: N, will be updated by callback function, [0] not used, [1] for load cell 1

        self.force_d = np.array([0.0,0.0,0.0])  # The desired force, will be updated by callback
        self.force_d_static = np.array([0.0,0.0,0.0])  # The desired force, will not be update by callback, will stay static during solving the 6 forces
        self.new_force_cmd = False

        # this is the desired tension on each cable based on the motor_dir and force_d at that time
        self.tension_desired = np.zeros(7) # unit: N, [0] not used, [1] for cable of force unit 1

        #based on desired tension and cable angle, canculate the desired load cell force reading
        self.load_cell_force_desired = np.zeros(7) # unit: N, [0] not used, [1] for the reading of force unit 1
        
        #this is the actual load cell force reading for calculating the applied force
        self.load_cell_force_cur = np.zeros(7) #np.array([0, 0, 0, 0, 0, 0, 0]) # unit: N, this value will be updated by callback function, [0] not used, [1] for the reading of force unit 1

        self.force_max = 5.0 * np.ones(7)  # [0] not used, [1] for motor 1 
        self.force_min =  -5.0 * np.ones(7)  # [0] not used, [1] for motor 1 

        self.last_solution = np.ones(6) * 2.2  # this is the the last solution,  [0] for motor 1 


        self.r2_tor_ctl = raven2_CRTK_torque_controller.raven2_crtk_torque_controller(name_space = ' ', robot_name_1 = 'arm1', robot_name_2 = 'arm2', grasper_name = 'grasp1', use_load_cell = True)
        self.tor_cmd = np.zeros(7)  # [0] is not used, [1] for motor 1

        self.y_force_only = True  # [TEST] This is only used for a temprary test, where only y axis force is given, by motor 4 and 5

        
        self.prev_tension_desired = None

        if not testing:
            self.__init_pub_sub()

        return None


    # setup ros publishers and subscribers
    def __init_pub_sub(self):

        # force and torque publisher, 
        # msg.position - (6,) x,y,z force and x, y, z torque (if applicable)
        # msg.velocity - (7,) desired torque command on motor 1-6, [0] is not used, [1] is motor 1
        self.__publisher_force_applied = rospy.Publisher('force_applied', sensor_msgs.msg.JointState, latch = True, queue_size = 1)
        # create a publisher that publish the torque command
        self.__publisher_torque_cmd = rospy.Publisher('torque_cmd', sensor_msgs.msg.JointState, latch = True, queue_size = 1)
        self.__subscriber_force_cmd = rospy.Subscriber('force_cmd', sensor_msgs.msg.JointState, self.__callback_force_cmd)
        # create a subscriber to get the end effector position
        self.__subscriber_ravenstate = rospy.Subscriber('ravenstate', raven_state, self.__callback_ravenstate)
        #create a subscriber to get the current load cell force reading
        self.__subscriber_load_cell_reading = rospy.Subscriber('/load_cell_forces', sensor_msgs.msg.JointState, self.__callback_load_cell_readings)

        return None

    def __callback_force_cmd(self, msg):
        self.force_d[0] = msg.position[0]
        self.force_d[1] = msg.position[1]
        self.force_d[2] = msg.position[2]
        self.new_force_cmd = True
        # print('force_cmd received')
        return None
    
    def __callback_ravenstate(self, msg):
        #isolate the loaction of gold arm grasper
        end_effector_pos_raw = np.array(msg.pos[0:3]) #unit: um
        #print("For debug - end_effector_pos_raw = ", end_effector_pos_raw, np.shape(end_effector_pos_raw))
        self.end_effector_loc = end_effector_pos_raw / 1000.0 #unit: mm
        # print("For debug - self.end_effector_loc = ", self.end_effector_loc)

    def __callback_load_cell_readings(self, msg):
        self.load_cell_force_cur[1:] = msg.position[:]
        # print("For debug - msg.position: ", msg.position)
        # print("For debug - self.load_cell_force_cur: ", self.load_cell_force_cur)

        return None


    def compute_motor_dir(self):
        self.motor_dir = self.end_effector_loc - self.motor_loc 
        for idx in range(1,7):
            self.motor_dir[idx] = self.motor_dir[idx] / np.linalg.norm(self.motor_dir[idx]) # normalize the direcrtion vector so that they all have a lenth of 1

    # calculate the angle between motor_dir and load cell direction
    def compute_cable_angles(self):
        #calculate dot product
        dot_product = np.sum(self.motor_dir * self.loadcell_dir, axis=1) # Compute the dot products between corresponding elements
        # print("For debug - dot_product = ",dot_product)

        #calculate magnitude of each vector
        motor_dir_mag = np.linalg.norm(self.motor_dir, axis=1)
        loadcell_dir_mag = np.linalg.norm(self.loadcell_dir, axis=1)
        # print("For debug - motor_dir_mag = ", motor_dir_mag)
        # print("For debug - loadcell_dir_mag = ", loadcell_dir_mag)

        #calculate the sin value of each unit
        cosine_val = dot_product / motor_dir_mag / loadcell_dir_mag
        # print("For debug - cosine_val = ", cosine_val, type(cosine_val))

        #calculate the angle in radient for each motor unit
        self.cable_load_cell_angles = np.arccos(cosine_val) #the angle value is not reliable, since the sign might be reverse
        # print("For debug - angles = ", self.cable_load_cell_angles)
        
        


    # force_cmd should be (3,), (x, y, z) component of the force in raven's frame 0
    # the force controller will always listen to topic 'force_cmd', and if there are commands published to this topic, input 'force_cmd' should be None
    def pub_force_cmd(self, force_cmd = None):
        if force_cmd is not None:
            self.force_d = force_cmd
            self.new_force_cmd = True

        if not self.new_force_cmd:
            return None
        
        if self.end_effector_loc == None:
            return None
        self.compute_motor_dir()
        self.compute_cable_angles()
        # bounds = [(2.2, 5.0)] * 6  # No bounds on the variables
        # bounds = [(0.5, 6.0)] * 6 #unit: N
        bounds = [(1.2, 6.0)] * 6 #unit: N

        self.force_d_static[:] =  self.force_d[:]
        #TODO:add a mechanism that use the previous result as initial guess
        if self.prev_tension_desired is None:
            self.prev_tension_desired = np.random.rand(6)   
        solution = minimize(self.objective, self.prev_tension_desired, bounds=bounds)
        # TODO end
        # solution = minimize(self.objective, np.random.rand(6), bounds=bounds)
        
        # this is the desired tension on each cable based on the motor_dir and force_d at that time
        # print("Solution: ", solution.x)  # [Test] Print the solution
        self.tension_desired[1:] = solution.x # unit: N
        # print("Solution: ", solution.x)  # [Test] Print the solution

        #based on desired tension and cable angle, canculate the desired load cell force reading
        self.load_cell_force_desired = self.tension_desired * np.cos(self.cable_load_cell_angles) 

        #convert the desired load cell force to torque command
        # (TODO: correct the variable name)the torque command will be converted desired load cell force again in torque controller
        self.tor_cmd[1:7] = self.load_cell_force_desired[1:7] * 10 #unit: N/mm
        
        
        #if only testing the 1D force, ignore the angle compensation
        if self.y_force_only:
            self.force_d_static[0] = 0 #unit: N
            self.force_d_static[2] = 0 #unit: N
            self.tor_cmd = np.zeros(7)
            if self.force_d_static[1] >=0:
                self.tor_cmd[4] = bounds[0][0]*10
                self.tor_cmd[5] = int(10* self.force_d_static[1] + bounds[0][0]*10)
            else:
                self.tor_cmd[5] = bounds[0][0]*10
                self.tor_cmd[4] = int(-10* self.force_d_static[1] + bounds[0][0]*10)
            # print("tor_cmd = ", self.tor_cmd)

        #TODO:publish the list of torque command to /torque_cmd topic
        #self.r2_tor_ctl.pub_torque_command_with_comp(self.tor_cmd) #original
        tor_cmd_msg = sensor_msgs.msg.JointState()
        tor_cmd_msg.position = self.tor_cmd
        self.__publisher_torque_cmd.publish(tor_cmd_msg)
        #TODO end
        
        # Applied force calculation
        load_cell_force_cur_static = self.load_cell_force_cur * 1 #fix the value, in case any update from callback function change the value
        # print("For debug - load_cell_force_cur_static:", load_cell_force_cur_static)
        #calculate the current cable tension using current cable angle
        cable_tension_cur = load_cell_force_cur_static[1:7] / np.cos(self.cable_load_cell_angles[1:7]) #unit: N
        # print("For debug - cable_tension_cur:", cable_tension_cur)
        tension_vec = np.zeros((6,1))
        # print("For debug - tension_vec:", tension_vec)
        tension_vec[:,0] =  cable_tension_cur.T #unit: N
        # print("For debug - tension_vec:", tension_vec)
        #calculate the applied force using the current load cell reading
        force_applied = np.sum(self.motor_dir[1:] * tension_vec, axis = 0)
        #prepare message for publish
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = np.zeros(6)
        if self.y_force_only:
            #if only testing 1 direction, just use the difference between two load cell force as applied force
            msg.position[1] = load_cell_force_cur_static[5] - load_cell_force_cur_static[4]
        else:
             msg.position[:] = force_applied.flat
        self.__publisher_force_applied.publish(msg)
        self.new_force_cmd = False
        #TODO: update the current tor_cmd to the prev_tension_desired
        self.prev_tension_desired = solution.x


        return None

    # Define the objective function
    def objective(self, force_value):
        force_vec = np.ones((6,1))
        force_vec[:,0] =  force_value.T
        #print(np.sum(self.motor_dir * force_vec))

        diff = np.sum(self.motor_dir[1:] * force_vec, axis = 0) - self.force_d_static
        obj = diff[0]**2 + diff[1]**2 + diff[2]**2    # this is much faster than using np.linalg.norm()
        #obj = np.linalg.norm(np.sum(self.motor_dir[1:] * force_vec, axis = 0) - self.force_d)
        #print(obj)
        return obj




    
