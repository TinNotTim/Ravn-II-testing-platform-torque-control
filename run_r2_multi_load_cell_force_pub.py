
import serial
import time
import numpy as np
import rospy
import sensor_msgs.msg

# [IMPT] Make sure that all motors have lose cable (0 force) when starting this code, will do zeroing first
calibration = True   # if true, will do calibration. If false, need to provide offset 'b'
calibration_num = 3000
buffer_clear_num = 3000 # this is to skip the readings at beginning to clear buffer
calibration_list = []

# # linear calibration factors -original
# a_1 = 6.534670690287095535e-06 #GUANG_CE_4
# a_2 = 6.578595718356571149e-06 #MAVIN_NA1
# a_3 = 6.717327973891286796e-06 #GUANG_CE_1
# a_4 = 6.950519714026019931e-06 #calt_dyx_306
# a_5 = 6.96586894668381e-06 #daysensor_dyx_306
# a_6 = 7.304287888493677691e-06 #GUANG_CE_3

# # linear calibration factors - swap the load cell for unit 1 and unit 6 10/19
# a_1 = 7.304287888493677691e-06#6.534670690287095535e-06 #GUANG_CE_4
# a_2 = 6.578595718356571149e-06 #MAVIN_NA1
# a_3 = 6.717327973891286796e-06 #GUANG_CE_1
# a_4 = 6.950519714026019931e-06 #calt_dyx_306
# a_5 = 6.96586894668381e-06 #daysensor_dyx_306
# a_6 = 6.534670690287095535e-06#7.304287888493677691e-06 #GUANG_CE_3

# linear calibration factors - base on 10/19 setting, confirm the factor
a_1 = 6.534670690287095535e-06 #GUANG_CE_4
a_2 = 6.578595718356571149e-06 #MAVIN_NA1
a_3 = 6.96586894668381e-06 #daysensor_dyx_306
a_4 = 7.187360987744082061e-06 #CZL_601
a_5 = 7.304287888493677691e-06 #GUANG_CE_3
a_6 = 6.950519714026019931e-06 #calt_dyx_306


a = [float(a_1), float(a_2), float(a_3), float(a_4), float(a_5), float(a_6)]
b = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

exp_decay_factor = 0.4#0.1#0.2
force_pre = np.zeros(6)

rospy.init_node('load_cell_driver', anonymous=True)
lcf_pub = rospy.Publisher('load_cell_forces', sensor_msgs.msg.JointState, queue_size=1)

ser = serial.Serial('/dev/ttyACM0', 115200)
rospy.sleep(3)

#--------Median Filter---------
#decide a window
window = 5
counter =  window
#create a queue as circular buffer
buffer = np.array([])

#--------Median Filter---------

while not rospy.is_shutdown():
            
    bt = ser.readline()         # read a byte string
    try:
        string_n = bt.decode()      # decode byte string into Unicode  
        string = string_n.rstrip() # remove \n and \r
    except:
        continue
    
    try:
        [reading_1, reading_2, reading_3, reading_4, reading_5, reading_6] = string.split('_')
    except:
        continue

    # publish raw reading
    raw_readings = [int(reading_1), int(reading_2), int(reading_3), int(reading_4), int(reading_5), int(reading_6)]
    # print("For debug - raw_readings = ", raw_readings)
    
    if calibration:
        cali_count = 0
        while cali_count < calibration_num + buffer_clear_num:
            if cali_count < buffer_clear_num:
                print(cali_count)
                cali_count += 1
                continue
            calibration_list.append(raw_readings)
            cali_count += 1
            print(cali_count)
        
        cali_avr = np.mean(np.array(calibration_list), axis = 0)
        for i in range(0,6):
            b[i] = -a[i] * cali_avr[i]
        calibration = False
        print(cali_avr)
        print(b)

        
  
    # publish calibrated force
    force_cur = np.array([raw_readings[0]*a[0] + b[0], raw_readings[1]*a[1] + b[1], raw_readings[2]*a[2] + b[2], raw_readings[3]*a[3] + b[3], raw_readings[4]*a[4] + b[4], raw_readings[5]*a[5] + b[5]])
    # print("For debug - force_cur size = ", force_cur.shape, type(force_cur))
    # for only one motor test -------------------
    # force_cur[0] = 0
    # force_cur[1] = 0
    # force_cur[2] = 0
    # force_cur[3] = 0
    # force_cur[5] = 0
    # for only one motor test -------------------

    #--------Median Filter---------
    #load the data into buffer
    buffer = np.append(buffer,force_cur)
    # buffer.append(force_cur)
    #before the buffer is filled, do nothing
    if counter > 0:
        counter -= 1
        continue


    #buffer has enough data
    #treat buffer as queue, remove the first element
    buffer = buffer.reshape(-1,6)
    buffer = buffer[1:]
    # print("For debug - buffer size = ", buffer.shape)
    #get the median of each reading
    data = np.array(buffer).T
    medians = np.array([np.median(subarray) for subarray in data])
    #--------Median Filter---------


    # force_filtered = medians
    force_filtered = exp_decay_factor * medians + (1-exp_decay_factor) * force_pre
    # force_filtered = exp_decay_factor * force_cur + (1-exp_decay_factor) * force_pre
    msg = sensor_msgs.msg.JointState()
    msg.header.stamp = rospy.Time.now()
    msg.position[:] = force_filtered.flat
    lcf_pub.publish(msg)
    #force_pre = force_cur
    force_pre = force_filtered


