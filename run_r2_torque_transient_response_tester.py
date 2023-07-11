"""
Raven 2 Control - Control software for the Raven II robot
Copyright (C) 2005-2023  Andrew Lewis, Yun-Hsuan Su, Haonan Peng, Blake Hannaford,
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
raven_keyboard_controller.py
Python controller for RAVEN II using CRTK API
date Apr 12, 2023
author Haonan Peng, Dun-Tin Chiang, Yun-Hsuan Su, Andrew Lewis, 
"""

"""
This piece of code is for testing the transient response of force unit. 
"""

import sys
import time
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import numpy as np
import matplotlib.pyplot as plt
import argparse
from raven2_CRTK_torque_controller_FB import raven2_crtk_torque_controller
import copy

class torque_transient_response_tester():

    def __init__(self):
        #general parameters
        self.testing_unit_index = 5
        self.pretension_force = 1 #unit: N
        self.cur_setpoint = 0 #unit: N
        self.setpoints = [] #for plotting 
        

        #parameters for step_response()
        self.wait_for_steady_state = False
        self.window_size = 30 # the number of readings to take for mean calculation
        self.window_reading_mean = 0
        self.steady_state_threshold = 0.01 
        self.step_response_force = 3 #unit: N

        #variable for load cell reading
        self.load_cell_force = 0 #unit N
        self.load_cell_forces = [] #for plotting
        self.load_cell_force_times = [] #for plotting
        self.start_time = 0
        self.cur_time = 0
        self.load_cell_start_record = False #to indicate the start of recording

        #TODO:Create a class variable to store current torque_cmd term value and a plotting list
        #variable for the current control signal
        self.cur_tor_cmd = 0 #unit: N/mm
        self.tor_cmds = [] #for plotting
        #TODO end

        #TODO: create variables and plotting list for current pid terms 
        self.cur_p_term = 0 
        self.cur_i_term = 0
        self.cur_d_term = 0
        self.p_terms = [] #for plotting
        self.i_terms = [] #for plotting
        self.d_terms = [] #for plotting
        #TODO end

        #create a torque controller object, just for getting the current PID gain
        self.torque_controller = raven2_crtk_torque_controller(name_space = ' ', robot_name_1 = 'arm1', robot_name_2 = 'arm2', grasper_name = 'grasp1', use_load_cell = True)
        self.pid_p = self.torque_controller.force_pid_p
        self.pid_i = self.torque_controller.force_pid_i
        self.pid_d = self.torque_controller.force_pid_d
        #print("For debug - self.pid_p = ", self.pid_p)
        del self.torque_controller

        #create publisher for torque command
        self.__publisher_torque_cmd = rospy.Publisher('torque_cmd', sensor_msgs.msg.JointState, latch = True, queue_size = 1)

        #create subscriber for load cell force
        self.subscriber_load_cell_force = rospy.Subscriber('/load_cell_forces', sensor_msgs.msg.JointState, self.__callback_load_cell_force)

        #TODO: Create a subscriber that subscribes to /arm2/servo_jp
        #subscriber for the current torque command after PID compensation
        self.__publisher_actual_torque_cmd = rospy.Subscriber('arm2/servo_jp', sensor_msgs.msg.JointState, self.__callback_actual_torque_cmd)
        #TODO end

        #TODO: Create a subscriber in transient tester subscribes to /pid_term_vals
        self.subscriber_pid_term_val = rospy.Subscriber('pid_term_vals', geometry_msgs.msg.PoseArray, self.__callback_pid_term_val)
        #TODO end

    # This is the most important callback function, since it will create all the plotting list with synchronized time
    def __callback_load_cell_force(self, msg):
        self.load_cell_force = msg.position[self.testing_unit_index-1]
        
        if self.load_cell_start_record:
            #record the force reading and time 
            if self.start_time == 0:
                rospy.logerr("Didn't specify the start time")
            self.cur_time = time.time()
            self.load_cell_force_times.append(self.cur_time - self.start_time)
            self.load_cell_forces.append(self.load_cell_force)

            #TODO:save current torque command in the plotting list during the recording period
            self.tor_cmds.append(self.cur_tor_cmd)
            #TODO end

            #TODO:save current setpoint in the plotting list during the recording period
            self.setpoints.append(self.cur_setpoint)
            #TODO end

            #TODO: save P, I and D term in separated plotting list during the recording period
            self.p_terms.append(self.cur_p_term)
            self.i_terms.append(self.cur_i_term)
            self.d_terms.append(self.cur_d_term)
            #TODO end

            #this is for step_response() to see if the steady state is reached
            if self.wait_for_steady_state:
                if len(self.load_cell_forces) < self.window_size:
                    print("Error, elements in the load_cell_forces are not enough for mean calculation")
                    return
                self.window_reading_mean = np.mean(self.load_cell_forces[-self.window_size:])
                #print("For debug - window_reading_mean = ", self.window_reading_mean)

    #TODO: Create a callback function for torque command, extract the corresponding command from JointState msg based on the testing unit index
    def __callback_actual_torque_cmd(self, msg):
        #the torque cmd will be store in msg.position
        #position is a list of size of 16, the force units occupy from 0 to 5
        self.cur_tor_cmd = msg.position[self.testing_unit_index-1]
    #TODO end

    #TODO: Create a callback function, extract the corresponding sublist from pid_msg based on the testing unit index
    def __callback_pid_term_val(self, msg):
        pid_term_for_unit = msg.poses[self.testing_unit_index].position
        self.cur_p_term = pid_term_for_unit.x 
        self.cur_i_term = pid_term_for_unit.y
        self.cur_d_term = pid_term_for_unit.z
    #TODO end

    #TODO: create a function that publish torque command and update the current setpoint
    def pub_force_cmd(self, desired_force):
        #update current setpoint
        self.cur_setpoint = desired_force

        #create publish message
        tor_cmd_msg = sensor_msgs.msg.JointState()
        tor_cmd_msg.position = np.zeros(7)
        tor_cmd_msg.position[self.testing_unit_index] = self.cur_setpoint * 10 # unit: N/mm, convert to  torque command

        #publish the message for pre-tension
        self.__publisher_torque_cmd.publish(tor_cmd_msg)
    #TODO end

    def pretension(self):
        rospy.loginfo("Pretension start...")
        self.pub_force_cmd(self.pretension_force)
        #wait for 3 seconds
        rospy.sleep(3.)
        rospy.loginfo("Pretension completed")      


    def step_response(self):
        #pretension the string
        self.pretension()

        #start recording
        rospy.loginfo("Start recording load cell reading")
        self.start_time = time.time()
        self.load_cell_start_record = True     

        #wait for 1 seconds
        rospy.sleep(1.)

        #publish the step response
        self.pub_force_cmd(self.step_response_force)

        #wait for steady state (3 seconds)
        rospy.sleep(3.)
        self.wait_for_steady_state = True
        #check if the load cell readings falls in threshold
        while True:
            total_time = time.time() - self.start_time
            increase_val = abs(self.step_response_force - self.pretension_force)
            error = abs(self.window_reading_mean - self.step_response_force) / increase_val 
            #print("For debug - error = ", error)

            if error < self.steady_state_threshold or total_time >= 8.0:
                break       
        
        #stop recording 
        rospy.loginfo("Stop recording load cell reading")
        self.load_cell_start_record = False
        self.wait_for_steady_state = False
        #print("For debug - load_cell_forces = ", self.load_cell_forces, len(self.load_cell_forces))
        #print("For debug - load_cell_force_times = ", self.load_cell_force_times, len(self.load_cell_force_times))

        #plot the load cell force
        self.plotter("step_response")

    def multi_setpoints(self):
        #pretension the string
        self.pretension()

        #start recording
        rospy.loginfo("Start recording load cell reading")
        self.start_time = time.time()
        self.load_cell_start_record = True     

        #wait for 1 seconds
        rospy.sleep(1.)

        #publish the step response
        self.pub_force_cmd(3.0)
        #wait for steady state (3 seconds)
        rospy.sleep(3.)
        #publish the step response
        self.pub_force_cmd(5.0)
        #wait for steady state (3 seconds)
        rospy.sleep(3.)
        #publish the step response
        self.pub_force_cmd(2.0)
        #wait for steady state (3 seconds)
        rospy.sleep(3.)
        #publish the step response
        self.pub_force_cmd(4.0)
        #wait for steady state (3 seconds)
        rospy.sleep(3.)
        #publish the step response
        self.pub_force_cmd(1.0)
        #wait for steady state (3 seconds)
        rospy.sleep(3.)

        #stop recording 
        rospy.loginfo("Stop recording load cell reading")
        self.load_cell_start_record = False
        #print("For debug - load_cell_forces = ", self.load_cell_forces, len(self.load_cell_forces))
        #print("For debug - load_cell_force_times = ", self.load_cell_force_times, len(self.load_cell_force_times))

        #plot the load cell force
        self.plotter("multi_setpoints")




        

    def plotter(self, plot_name="test"):
        fig, ax = plt.subplots(3)
        fig.suptitle(plot_name)

        #plot setpoints, load cell readings and pid terms in first graph
        ax[0].plot(self.load_cell_force_times, self.setpoints, label="setpoint")
        ax[0].plot(self.load_cell_force_times, self.load_cell_forces, label="load cell force")
        ax[0].set(xlabel='time (s)', ylabel='force reading (N)')
        ax[0].set_title("Output signal")

        #display the pid parameter
        # add text box for the statistics
        stats = ('Kp = %.1f\nKi = %.1f\nKd = %.1f\nUnit:%i'%(self.pid_p, self.pid_i, self.pid_d, self.testing_unit_index))
        bbox = dict(boxstyle='round', fc='blanchedalmond', ec='orange', alpha=0.5)
        ax[0].text(0.95, 0.07, stats, fontsize=9, bbox=bbox,transform=ax[0].transAxes, horizontalalignment='right')
        ax[0].set(ylim=(0, 5))
        ax[0].legend()

        #plot pid terms
        ax[1].plot(self.load_cell_force_times, self.p_terms, label="p term")
        ax[1].plot(self.load_cell_force_times, self.i_terms, label="i term")
        ax[1].plot(self.load_cell_force_times, self.d_terms, label="d term")
        ax[1].set(xlabel='time (s)', ylabel='torque command (N/mm)')
        ax[1].set_title("PID values")
        ax[1].legend()

        #plot control signal
        ax[2].plot(self.load_cell_force_times, self.tor_cmds)
        ax[2].set(xlabel='time (s)', ylabel='torque command (N/mm)')
        ax[2].set_title("Control Signal")

        #mng = plt.get_current_fig_manager()
        #mng.full_screen_toggle()

        figure = plt.gcf() # get current figure
        figure.set_size_inches(8, 6)


        save_dir = '/home/supernova/raven2_CRTK_Python_controller/torque_controller/transient_response_test_fig/'
        file_name = plot_name + 'kp%.1f_ki%.1f_kd%.1f_unit%i'%(self.pid_p, self.pid_i, self.pid_d, self.testing_unit_index)
        fig.savefig(save_dir + file_name + ".png", dpi=100)
        plt.show()


# def get_args():
#     parser = argparse.ArgumentParser(description='script for testing planners')

#     parser.add_argument('-f', '--scene', type=str, default='2dof_robot_arm',
#                         help='The environment to plan on, 2dof_robot_arm, 3dof_robot_arm, car')
#     return parser.parse_args()


if __name__ == '__main__':
        rospy.init_node('transient_response_tester', anonymous=True)
        rospy.loginfo("Node is created")

        tester = torque_transient_response_tester()
        #tester.plotter()
        #tester.step_response()
        tester.multi_setpoints()
