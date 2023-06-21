
import serial
import time
import numpy as np
import rospy
import sensor_msgs.msg

# linear calibration factors
a_1 = 2.163318232246724780e-05
b_1 = -1.385993314600030946e+00

a_2 = 2.163318232246724780e-05
b_2 = -1.385993314600030946e+00

a_3 = 2.163318232246724780e-05
b_3 = -1.385993314600030946e+00

a_4 = 2.163318232246724780e-05
b_4 = -1.385993314600030946e+00

a_5 = 2.163318232246724780e-05
b_5 = -1.385993314600030946e+00

a_6 = 2.163318232246724780e-05
b_6 = -1.385993314600030946e+00

exp_decay_factor = 0.2
force_pre = np.zeros(6)

rospy.init_node('load_cell_driver', anonymous=True)
lcf_pub = rospy.Publisher('load_cell_forces', sensor_msgs.msg.JointState, queue_size=1)

ser = serial.Serial('/dev/ttyACM0', 115200)
rospy.sleep(3)

while not rospy.is_shutdown():
            
    b = ser.readline()         # read a byte string
    try:
        string_n = b.decode()      # decode byte string into Unicode  
        string = string_n.rstrip() # remove \n and \r
    except:
        continue
    
    try:
        [reading_1, reading_2, reading_3, reading_4, reading_5, reading_6] = string.split('_')
    except:
        continue

    # publish raw reading
    raw_readings = np.array([int(reading_1), int(reading_2), int(reading_3), int(reading_4), int(reading_5), int(reading_6)])
  
    # publish calibrated force
    force_cur = np.array([raw_readings[0]*a_1 + b_1, raw_readings[1]*a_2 + b_2, raw_readings[2]*a_3 + b_3, raw_readings[3]*a_4 + b_4, raw_readings[4]*a_5 + b_5, raw_readings[5]*a_6 + b_6])

    # for only one motor test -------------------
    force_cur[0] = 0
    force_cur[1] = 0
    force_cur[2] = 0
    force_cur[3] = 0
    force_cur[5] = 0
    # for only one motor test -------------------


    force_filtered = exp_decay_factor * force_cur + (1-exp_decay_factor) * force_pre
    msg = sensor_msgs.msg.JointState()
    msg.header.stamp = rospy.Time.now()
    msg.position[:] = force_filtered.flat
    lcf_pub.publish(msg)
    force_pre = force_cur


