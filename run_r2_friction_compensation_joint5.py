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



joint = 5
target_torque = 20.0  #assume this parameter is assignend by other higher controller 
control_torque = target_torque
max_torque = 50.0 

#these flags indicate the on/off of the compensation
compensation_master = 1
coulomb_compensation = 1
viscous_compensation = 1

#define compensation factor
coulomb_factor = 0.5
viscous_factor = 0.001


rospy.init_node('force_unit_joint5', anonymous=True)
rospy.loginfo("Node is created")

#set the rate to 100 Hz
r = rospy.Rate(100)

r2_tor_ctl = raven2_CRTK_torque_controller.raven2_crtk_torque_controller(name_space = ' ', robot_name_1 = 'arm1', robot_name_2 = 'arm2', grasper_name = 'grasp1')

if compensation_master:
    rospy.loginfo("Friction compensation is on")
    while(True):
        try:
            motor_pose = r2_tor_ctl.ravenstate_cur.mpos[13]
            motor_velocity = r2_tor_ctl.ravenstate_cur.mvel[13]
            rospy.loginfo("Current vel = %f, current torque command = %i", motor_velocity, control_torque)
        except:
            rospy.loginfo("No ravenstate yet")
            continue

        if coulomb_compensation:
            #define the motor behavior based on velocity
            if motor_velocity > 0.2: #forward drive
                control_torque = target_torque + target_torque * coulomb_factor

            elif motor_velocity < -0.2: #backdrive
                control_torque = target_torque - target_torque * coulomb_factor
        
            else: #static
                control_torque = target_torque



        if viscous_compensation:
            control_torque += np.clip(motor_velocity * viscous_factor, -5, 5)

        #only send out control command if the value change
        if control_torque != target_torque and control_torque <= max_torque and control_torque >= -max_torque:
            cmd = np.zeros((16))
            cmd[joint] = control_torque
            r2_tor_ctl.pub_torque_command(cmd)
            rospy.loginfo("command sent!")
        
        r.sleep()


else:
    rospy.loginfo("Friction compensation is off")
    cmd = np.zeros((16))
    cmd[joint] = control_torque
    while(True):
        r2_tor_ctl.pub_torque_command(cmd)
        r.sleep()

        

sys.exit()
