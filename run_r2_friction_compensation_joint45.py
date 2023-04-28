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
This piece of code is for testing the single force unit. 
The code reads in raven state, and compensates coulomb and viscous friction based on motor velocity 
"""

import sys
import time
import utils_r2_torque_keyboard_controller as utils
import rospy
import numpy as np
import raven2_CRTK_torque_controller
import copy



# joint = 4 #for crtk, use joint to index
# offset = 7
# if joint < 4:
#     raven_state_index = offset + joint #for ravenstate, use
# else:
#     raven_state_index = offset + joint + 1
 
target_torques = np.array([0, 0, 0, 0, 20.0, 20.0, 0, 0])  #assume these parameters are assignend by other higher controller 
#control_torques = copy.deepcopy(target_torques)
control_torques = np.zeros(8)
max_torque = 50.0 

#these flags indicate the on/off of the compensation
compensation_master = 1
coulomb_compensation = 1
viscous_compensation = 1

#define compensation factor
coulomb_factor = 0.5
viscous_factor = 0.001


rospy.init_node('force_unit_joint45', anonymous=True)
rospy.loginfo("Node is created")

#set the rate to 100 Hz
r = rospy.Rate(100)

r2_tor_ctl = raven2_CRTK_torque_controller.raven2_crtk_torque_controller(name_space = ' ', robot_name_1 = 'arm1', robot_name_2 = 'arm2', grasper_name = 'grasp1')



if compensation_master:
    rospy.loginfo("Friction compensation is on")
    while(True):
        try:
            #motor_poses = r2_tor_ctl.ravenstate_cur.mpos
            motor_velocities = r2_tor_ctl.ravenstate_cur.mvel
            print("Current vel = ", motor_velocities)
            #rospy.loginfo("Test - got ravenstate")
            print("Current command = ", control_torques)
        except:
            rospy.loginfo("No ravenstate yet")
            continue

        for i in range(len(target_torques)):
		#calculate the control torque for each joint
	        if coulomb_compensation:
	            #define the motor behavior based on velocity
	            if motor_velocities[i+8] > 0.2: #forward drive
	                control_torques[i] = target_torques[i] + target_torques[i] * coulomb_factor

	            elif motor_velocities[i+8] < -0.2: #backdrive
	                control_torques[i] = target_torques[i] - target_torques[i] * coulomb_factor
	        
	            else: #static
	                control_torques[i] = target_torques[i]



	        if viscous_compensation:
	            control_torques[i] += np.clip(motor_velocities[i+8] * viscous_factor, -5, 5)

        cmd = np.append(np.clip(control_torques, -max_torque, max_torque), np.zeros(8))
        r2_tor_ctl.pub_torque_command(cmd)
        print("command = ", cmd)
        rospy.loginfo("command sent!")
        r.sleep()


else:
    rospy.loginfo("Friction compensation is off")
    control_torques = target_torques*1
    cmd = np.append(control_torques ,np.zeros(8))
    while(True):
        r2_tor_ctl.pub_torque_command(cmd)
        r.sleep()

        

sys.exit()
