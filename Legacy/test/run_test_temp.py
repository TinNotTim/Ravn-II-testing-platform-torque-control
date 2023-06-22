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
import sys
import time
import rospy
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sensor_msgs.msg

    
rospy.init_node('raven_trajectory_follower_with_force', anonymous=True)
force_cmd_publisher = rospy.Publisher('force_cmd', sensor_msgs.msg.JointState, latch = True, queue_size = 1)  # create torque cmd publisher

#set the rate to 100 Hz
r = rospy.Rate(100)



while not rospy.is_shutdown():
    force_msg = sensor_msgs.msg.JointState()
    force_msg.position[:] = np.zeros(3)
    force_msg.position[1] = 2.0  # this is to match range [50, 115] degree to range [-2.8, +2.8] N froce
    force_cmd_publisher.publish(force_msg)
    r.sleep()



