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
This piece of code is for testing the transient response of all 6 force unit. 
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
#from run_r2_multi_load_cell_force_pub import exp_decay_factor
import copy

class torque_transient_response_tester():

    def __init__(self):
        #print("For debug - init")
        #general parameters
        #from 1 to 6, fill in the units for testing. only put in each unit once
        self.testing_unit_indices = [4,5]
        self.testing_unit_index = None
        self.pretension_force = 1 #unit: N
        self.cur_setpoint = np.zeros(6) #unit: N
        self.setpoints = [[],[],[],[],[],[]] #for plotting 
        

        #parameters for step_response()
        self.wait_for_steady_state = False
        self.window_size = 30 # the number of readings to take for mean calculation
        self.window_reading_mean = 0
        self.steady_state_threshold = 0.01 
        self.step_response_force = 3 #unit: N

        #parameters for disturbance_test()
        self.disturbance_test_force = 3 #unit: N
        self.disturbance_test_time = 30 #unit: second

        #parameters for generate sine and cosine wave
        self.num_points = 50
        self.amplitude = 2 

        #variable for load cell reading
        self.load_cell_force = np.zeros(6) #unit N
        self.load_cell_forces = [[],[],[],[],[],[]] #for plotting
        self.load_cell_force_times = [] #for plotting, only have one list
        self.start_time = 0
        self.cur_time = 0
        self.load_cell_start_record = False #to indicate the start of recording

        #TODO:Create a class variable to store current torque_cmd term value and a plotting list
        #variable for the current control signal
        self.cur_tor_cmd = np.zeros(6) #unit: N/mm
        self.tor_cmds = [[],[],[],[],[],[]] #for plotting
        #TODO end

        #TODO: create variables and plotting list for current pid terms 
        self.cur_p_term = np.zeros(6) 
        self.cur_i_term = np.zeros(6) 
        self.cur_d_term = np.zeros(6) 
        self.p_terms = [[],[],[],[],[],[]] #for plotting
        self.i_terms = [[],[],[],[],[],[]] #for plotting
        self.d_terms = [[],[],[],[],[],[]] #for plotting
        #TODO end

        #create a torque controller object, just for getting the current PID gain
        self.torque_controller = raven2_crtk_torque_controller(name_space = ' ', robot_name_1 = 'arm1', robot_name_2 = 'arm2', grasper_name = 'grasp1', use_load_cell = True)
        self.pid_p = self.torque_controller.force_pid_p
        self.pid_i = self.torque_controller.force_pid_i
        self.pid_d = self.torque_controller.force_pid_d
        #print("For debug - self.pid_p = ", self.pid_p)
        del self.torque_controller

        self.exp_decay_factor = 0.1 #0.2

        #create publisher for torque command
        self.__publisher_torque_cmd = rospy.Publisher('torque_cmd', sensor_msgs.msg.JointState, latch = True, queue_size = 1)

        #create subscriber for load cell force
        self.subscriber_load_cell_force = rospy.Subscriber('/load_cell_forces', sensor_msgs.msg.JointState, self.__callback_load_cell_force)

        #TODO: Create a subscriber that subscribes to /arm2/servo_jp
        #subscriber for the current torque command after PID compensation
        self.__subscriber_actual_torque_cmd = rospy.Subscriber('arm2/servo_jp', sensor_msgs.msg.JointState, self.__callback_actual_torque_cmd)
        #TODO end

        #TODO: Create a subscriber in transient tester subscribes to /pid_term_vals
        self.subscriber_pid_term_val = rospy.Subscriber('pid_term_vals', geometry_msgs.msg.PoseArray, self.__callback_pid_term_val)
        #TODO end

    # This is the most important callback function, since it will create all the plotting list with synchronized time
    def __callback_load_cell_force(self, msg):
        self.load_cell_force[:] = msg.position[:]
        
        if self.load_cell_start_record:
            #record the force reading and time 
            if self.start_time == 0:
                rospy.logerr("Didn't specify the start time")
            self.cur_time = time.time()
            self.load_cell_force_times.append(self.cur_time - self.start_time)

            for unit_index in self.testing_unit_indices:
                self.load_cell_forces[unit_index - 1].append(self.load_cell_force[unit_index - 1])

                #TODO:save current torque command in the plotting list during the recording period
                self.tor_cmds[unit_index - 1].append(self.cur_tor_cmd[unit_index - 1])
                #TODO end

                #TODO:save current setpoint in the plotting list during the recording period
                self.setpoints[unit_index - 1].append(self.cur_setpoint[unit_index - 1])
                #TODO end

                #TODO: save P, I and D term in separated plotting list during the recording period
                self.p_terms[unit_index - 1].append(self.cur_p_term[unit_index - 1])
                self.i_terms[unit_index - 1].append(self.cur_i_term[unit_index - 1])
                self.d_terms[unit_index - 1].append(self.cur_d_term[unit_index - 1])
                #TODO end

            # #this is for step_response() to see if the steady state is reached
            # if self.wait_for_steady_state:
            #     if len(self.load_cell_forces) < self.window_size:
            #         print("Error, elements in the load_cell_forces are not enough for mean calculation")
            #         return
            #     self.window_reading_mean = np.mean(self.load_cell_forces[-self.window_size:])
            #     #print("For debug - window_reading_mean = ", self.window_reading_mean)

    
    def __callback_actual_torque_cmd(self, msg):
        #the torque cmd will be store in msg.position: [0, tor1, tor2, tor3, tor4, tor5, tor6, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.cur_tor_cmd[:] = msg.position[1:7]
    

    
    def __callback_pid_term_val(self, msg):
        for pose in msg.poses[1:7]:
            pid_term_for_units.append(pose.position)

        for unit_index in self.testing_unit_indices:
            self.cur_p_term[unit_index - 1] = pid_term_for_units[unit_index - 1].x
            self.cur_i_term[unit_index - 1] = pid_term_for_units[unit_index - 1].y
            self.cur_d_term[unit_index - 1] = pid_term_for_units[unit_index - 1].z

    #the desired_force format: [force1, force2, force3, force4, force5, force6]
    def pub_force_cmd(self, desired_force):
        #update current setpoint
        for unit_index in self.testing_unit_indices:
            self.cur_setpoint[unit_index - 1] = desired_force[unit_index - 1]

        #create publish message
        tor_cmd_msg = sensor_msgs.msg.JointState()
        tor_cmd_msg.position = np.zeros(7)
        tor_cmd_msg.position[1:] = desired_force[:] * 10 # unit: N/mm, convert to  torque command

        #publish the message for pre-tension
        self.__publisher_torque_cmd.publish(tor_cmd_msg)
    #TODO end

    def gen_sine_wave(self):
        #define a discrete sine wave 
        start = 0
        stop = 2 * np.pi
        # Define the number of data points
        # num_points = 50

        # Generate the time axis
        t = np.linspace(start, stop, self.num_points)

        # Generate the discrete sine wave
        offset = self.pretension_force + self.amplitude
        sine_wave_setpoints = offset + self.amplitude * np.sin(t)
        #print("For debug - t = ", t)
        #print("For debug - setpoints = ", sine_wave_setpoints)
        return sine_wave_setpoints
    
    def gen_cosine_wave(self):
        #define a discrete sine wave 
        start = 0
        stop = 2 * np.pi
        # Define the number of data points
        # num_points = 50

        # Generate the time axis
        t = np.linspace(start, stop, self.num_points)

        # Generate the discrete cosine wave
        offset = self.pretension_force + self.amplitude
        cosine_wave_setpoints = offset + self.amplitude * np.cos(t)
        #print("For debug - t = ", t)
        #print("For debug - setpoints = ", cosine_wave_setpoints)
        return cosine_wave_setpoints

    def pretension(self):
        rospy.loginfo("Pretension start...")
        #create a zero array
        desired_force = np.zeros(6)

        for unit_index in self.testing_unit_indices:
            desired_force[unit_index - 1] = self.pretension_force

        self.pub_force_cmd(desired_force)
        #wait for 3 seconds
        rospy.sleep(3.)
        rospy.loginfo("Pretension completed")      


    # def step_response(self):
    #     #pretension the string
    #     self.pretension()

    #     #start recording
    #     rospy.loginfo("Start recording load cell reading")
    #     self.start_time = time.time()
    #     self.load_cell_start_record = True     

    #     #wait for 1 seconds
    #     rospy.sleep(1.)

    #     #publish the step response
    #     self.pub_force_cmd(self.step_response_force)

    #     #wait for steady state (3 seconds)
    #     rospy.sleep(3.)
    #     self.wait_for_steady_state = True
    #     #check if the load cell readings falls in threshold
    #     while True:
    #         total_time = time.time() - self.start_time
    #         increase_val = abs(self.step_response_force - self.pretension_force)
    #         error = abs(self.window_reading_mean - self.step_response_force) / increase_val 
    #         #print("For debug - error = ", error)

    #         if error < self.steady_state_threshold or total_time >= 8.0:
    #             break       
        
    #     #stop recording 
    #     rospy.loginfo("Stop recording load cell reading")
    #     self.load_cell_start_record = False
    #     self.wait_for_steady_state = False
    #     #print("For debug - load_cell_forces = ", self.load_cell_forces, len(self.load_cell_forces))
    #     #print("For debug - load_cell_force_times = ", self.load_cell_force_times, len(self.load_cell_force_times))

    #     #plot the load cell force
    #     self.plotter("step_response")

    # #huge change between setpoints, might oscillate due to the delay in I term
    # def multi_setpoints(self): 
    #     #pretension the string
    #     self.pretension()

    #     #start recording
    #     rospy.loginfo("Start recording load cell reading")
    #     self.start_time = time.time()
    #     self.load_cell_start_record = True     

    #     #wait for 1 seconds
    #     rospy.sleep(1.)

    #     #publish the step response
    #     self.pub_force_cmd(3.0)
    #     #wait for steady state (3 seconds)
    #     rospy.sleep(3.)
    #     #publish the step response
    #     self.pub_force_cmd(5.0)
    #     #wait for steady state (3 seconds)
    #     rospy.sleep(3.)
    #     #publish the step response
    #     self.pub_force_cmd(2.0)
    #     #wait for steady state (3 seconds)
    #     rospy.sleep(3.)
    #     #publish the step response
    #     self.pub_force_cmd(4.0)
    #     #wait for steady state (3 seconds)
    #     rospy.sleep(3.)
    #     #publish the step response
    #     self.pub_force_cmd(1.0)
    #     #wait for steady state (3 seconds)
    #     rospy.sleep(3.)

    #     #stop recording 
    #     rospy.loginfo("Stop recording load cell reading")
    #     self.load_cell_start_record = False
    #     #print("For debug - load_cell_forces = ", self.load_cell_forces, len(self.load_cell_forces))
    #     #print("For debug - load_cell_force_times = ", self.load_cell_force_times, len(self.load_cell_force_times))

    #     #plot the load cell force
    #     self.plotter("multi_setpoints")

    # def multi_setpoints_sine(self):
    #     #define a discrete sine wave 
    #     start = 0
    #     stop = 2 * np.pi
    #     # Define the number of data points
    #     num_points = 50

    #     # Generate the time axis
    #     t = np.linspace(start, stop, num_points)

    #     # Generate the discrete sine wave
    #     amplitude = 2
    #     frequency = 1
    #     offset = self.pretension_force + amplitude
    #     sine_wave_setpoints = offset + amplitude * np.sin(t)
    #     #print("For debug - t = ", t)
    #     #print("For debug - setpoints = ", sine_wave_setpoints)

    #     #pretension the string
    #     self.pretension()

    #     #start recording
    #     rospy.loginfo("Start recording load cell reading")
    #     self.start_time = time.time()
    #     self.load_cell_start_record = True     

    #     #wait for 1 seconds
    #     rospy.sleep(1.)


    #     # use a while loop to gently change the setpoint
    #     current_force = self.pretension_force
    #     for setpoint in sine_wave_setpoints:
    #         self.pub_force_cmd(setpoint)
    #         rospy.sleep(1.)

    #     #stop recording 
    #     rospy.loginfo("Stop recording load cell reading")
    #     self.load_cell_start_record = False

    #     #plot the load cell force
    #     self.plotter("multi_setpoints_sine")

    
    # #have the raven run random trajectory, and let force unit output a constant torque, see how well it maintains it
    # def disturbance_test(self):
    #     #pretension the string
    #     self.pretension()

    #     #user_input = input("Start the random trajectory, then enter any key")

    #     #start recording
    #     rospy.loginfo("Start recording load cell reading")
    #     self.start_time = time.time()
    #     self.load_cell_start_record = True     

    #     #wait for 1 seconds
    #     rospy.sleep(1.)

    #     self.pub_force_cmd(self.disturbance_test_force)
    #     rospy.sleep(self.disturbance_test_time)

    #     #stop recording 
    #     rospy.loginfo("Stop recording load cell reading")
    #     self.load_cell_start_record = False

    #     #plot the load cell force
    #     self.plotter("disturbance_test")

    

    #have the raven run random trajectory, and let force unit output a sine wave torque, see how well it follows it
    #let the unit4 do cosine wave, and let unit5 sine wave
    def multi_setpoints_sine_with_disturb(self):
        self.setpoints[4] = self.gen_cosine_wave()
        self.setpoints[5] = self.gen_sine_wave()

        #pretension the string
        self.pretension()

        #start recording
        rospy.loginfo("Start recording load cell reading")
        self.start_time = time.time()
        self.load_cell_start_record = True     

        #wait for 1 seconds
        rospy.sleep(1.)

        
        for i in range(self.num_points):
            desired_force = np.zeros(6)
            for unit_index in self.testing_unit_indices:
                desired_force[unit_index -1] = self.setpoints[unit_index -1][i]
            self.pub_force_cmd(desired_force)
            rospy.sleep(1.)

        # use a while loop to gently change the setpoint
        for setpoint in sine_wave_setpoints:
            self.pub_force_cmd(setpoint)
            rospy.sleep(1.)

        #stop recording 
        rospy.loginfo("Stop recording load cell reading")
        self.load_cell_start_record = False

        #plot the load cell force
        self.plotter("multi_setpoints_sine_with_disturb")




        

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
        stats = ('Kp = %.1f\nKi = %.1f\nKd = %.1f\nDecay factor = %.1f\nUnit:%i'%(self.pid_p, self.pid_i, self.pid_d, self.exp_decay_factor, self.testing_unit_index))
        bbox = dict(boxstyle='round', fc='blanchedalmond', ec='orange', alpha=0.5)
        ax[0].text(0.95, 0.07, stats, fontsize=9, bbox=bbox,transform=ax[0].transAxes, horizontalalignment='right')
        ax[0].set(ylim=(0, 6))
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
        file_name = plot_name + '_kp%.1f_ki%.1f_kd%.1f_unit%i'%(self.pid_p, self.pid_i, self.pid_d, self.testing_unit_index)
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
        #tester.multi_setpoints()
        #tester.multi_setpoints_sine()
        #tester.disturbance_test()
        tester.multi_setpoints_sine_with_disturb()
