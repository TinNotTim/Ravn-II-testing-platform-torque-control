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
import sensor_msgs.msg
import numpy as np
import matplotlib.pyplot as plt
import argparse
from raven2_CRTK_torque_controller_FB import raven2_crtk_torque_controller

class torque_transient_response_tester():

    def __init__(self):
        #create parameters
        self.testing_unit_index = 5
        self.pretension_force = 1 #unit: N
        self.step_response_force = 3 #unit: N
        self.wait_for_steady_state = False
        self.window_size = 10 # the number of readings to take for mean calculation
        self.window_reading_mean = 0
        self.steady_state_threshold = 0.02 

        #variable for load cell reading
        self.load_cell_force = 0 #unit N
        self.load_cell_forces = []
        self.load_cell_force_times = []
        self.start_time = 0
        self.cur_time = 0
        self.load_cell_start_record = False #to indicate the start of recording

        #create a torque controller object, just for getting the PID gain
        self.torque_controller = raven2_crtk_torque_controller(name_space = ' ', robot_name_1 = 'arm1', robot_name_2 = 'arm2', grasper_name = 'grasp1', use_load_cell = True)
        self.pid_p = self.torque_controller.force_pid_p
        self.pid_i = self.torque_controller.force_pid_i
        self.pid_d = self.torque_controller.force_pid_d
        print("For debug - self.pid_p = ", self.pid_p)
        del self.torque_controller

        #create publisher for torque command
        self.__publisher_torque_cmd = rospy.Publisher('torque_cmd', sensor_msgs.msg.JointState, latch = True, queue_size = 1)

        #create subscriber for load cell force
        self.subscriber_load_cell_force = rospy.Subscriber('/load_cell_forces', sensor_msgs.msg.JointState, self.__callback_load_cell_force)


    def __callback_load_cell_force(self, msg):
        self.load_cell_force = msg.position[self.testing_unit_index-1]

        if self.load_cell_start_record:
            #record the force reading and time 
            if self.start_time == 0:
                rospy.logerr("Didn't specify the start time")
            self.cur_time = time.time()
            self.load_cell_force_times.append(self.cur_time - self.start_time)
            self.load_cell_forces.append(self.load_cell_force)

            if self.wait_for_steady_state:
                if len(self.load_cell_forces) < self.window_size:
                    print("Error, elements in the load_cell_forces are not enough for mean calculation")
                    continue
                self.window_reading_mean = np.mean(self.load_cell_forces[-self.window_size:])

    def pretension(self):
        rospy.loginfo("Pretension start...")
        #create publish message
        tor_cmd_msg = sensor_msgs.msg.JointState()
        tor_cmd_msg.position = np.zeros(7)
        tor_cmd_msg.position[self.testing_unit_index] = self.pretension_force * 10 # unit: N/mm, convert to  torque command

        #publish the message for pre-tension
        self.__publisher_torque_cmd.publish(tor_cmd_msg)

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
        tor_cmd_msg = sensor_msgs.msg.JointState()
        tor_cmd_msg.position = np.zeros(7)
        tor_cmd_msg.position[self.testing_unit_index] = self.step_response_force * 10 # unit: N/mm, convert to  torque command
        self.__publisher_torque_cmd.publish(tor_cmd_msg)

        #wait for steady state (3 seconds)
        self.wait_for_steady_state = True
        #check if the load cell readings falls in threshold
        while True:
            error = abs(self.window_reading_mean - self.step_response_force) / self.step_response_force 

            if error < self.steady_state_threshold:
                break       
        
        #stop recording 
        rospy.loginfo("Stop recording load cell reading")
        self.load_cell_start_record = False
        self.wait_for_steady_state = False
        #print("For debug - load_cell_forces = ", self.load_cell_forces, len(self.load_cell_forces))
        #print("For debug - load_cell_force_times = ", self.load_cell_force_times, len(self.load_cell_force_times))

        #plot the load cell force
        self.plotter("step_response")

        

    def plotter(self, plot_name="test"):
        fig, ax = plt.subplots()
        ax.plot(self.load_cell_force_times, self.load_cell_forces)

        ax.set(xlabel='time (s)', ylabel='force reading (N)', title=plot_name)
        #ax.grid()
        #display the initial force and desired force
        plt.axhline(y=self.pretension_force, color='b', linestyle='-')
        plt.axhline(y=self.step_response_force, color='r', linestyle='-')

        #display the pid parameter
        #ax.text(0.85, 0.95, f'Kp: {self.pid_p}, Ki: {self.pid_i}, Kd: {self.pid_d}, Force Unit: {self.testing_unit_index}', transform=ax.transAxes, fontsize=10, verticalalignment='top', bbox=dict(facecolor='white', edgecolor='black', boxstyle='round'))
        #ax.text(0.85, 0.95, f'Kp: {self.pid_p}, Ki: {self.pid_i}, Kd: {self.pid_d}, Force Unit: {self.testing_unit_index}')

        save_dir = '/home/supernova/raven2_CRTK_Python_controller/torque_controller/transient_response_test_fig/'
        file_name = plot_name + f'kp{self.pid_p}_ki{self.pid_i}_kd{self.pid_d}_unit{self.testing_unit_index}'
        fig.savefig(save_dir + file_name + ".png")
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
        tester.step_response() 
