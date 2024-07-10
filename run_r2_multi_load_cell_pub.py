
import serial
import time
import numpy as np
import rospy
import sensor_msgs.msg
import struct
from copy import deepcopy


rospy.init_node('load_cell_driver', anonymous=True)
lc_pub = rospy.Publisher('load_cells', sensor_msgs.msg.JointState, queue_size=1)

ser = serial.Serial('/dev/ttyACM0', 115200)

ser.reset_input_buffer()


rospy.sleep(3)

loadcell_num = 6

# Communication handshake
INIT = 'I'
SIG_PC = 'S'
ACK = 'A'

while True:
    if ser.in_waiting > 0:
        received = ser.read(size=1)
        # Arduino has done initialization
        if received == INIT:
            ser.reset_output_buffer()
            # Signal the arduino to start data transmission
            ser.write(SIG_PC)
            break





while not rospy.is_shutdown():
    
    if ser.in_waiting >= loadcell_num * 4:
        byte_arry = ser.read(size = loadcell_num * 4) # read a byte array of 6 int32
        # try:
        #     print(list(byte_arry.decode()))
        # # print(len(byte_arry))
        # except:
        #     pass

        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position[:] = [None] * loadcell_num

        # Decode the byte array into int32
        for i in range(loadcell_num):
            # prepare buffer for unpacked
            buff = deepcopy(byte_arry[(0 + i * 4) : (4 + i * 4)])
            buff_int = [ord(byte) for byte in buff]
            print(buff_int)
            # unpack every 4 byte in the array into an int( little endian)
            # reading = struct.unpack('<i', buff)[0]
            reading = (buff_int[0] | (buff_int[1]<<8) | (buff_int[2]<<16) | (buff_int[3]<<24))
            # print(reading)
            msg.position[i] = float(reading)
        
        lc_pub.publish(msg)


