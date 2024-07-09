
import serial
import time
import numpy as np
import rospy
import sensor_msgs.msg


rospy.init_node('load_cell_driver', anonymous=True)
lc_pub = rospy.Publisher('load_cells', sensor_msgs.msg.JointState, queue_size=1)

ser = serial.Serial('/dev/ttyACM0', 115200)
rospy.sleep(3)

loadcell_num = 6

while not rospy.is_shutdown():
    
    if ser.in_waiting >= loadcell_num * size(int32):
        byte_arry = ser.read(loadcell_num * size(int32))         # read a byte array of 6 int32

        # Decode the byte array into int32
        for i in range(loadcell_num):
            


        try:
            string_n = b.decode()      # decode byte string into Unicode  
            string = string_n.rstrip() # remove \n and \r
        except:
            continue
        
        try:
            [reading_1, reading_2, reading_3, reading_4, reading_5, reading_6] = string.split('_')
        except:
            continue


        # new_line = np.array([float(reading_1), float(reading_2), float(reading_3), float(reading_4), float(reading_5), float(reading_6)])
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position[:] = new_line.flat
        lc_pub.publish(msg)


