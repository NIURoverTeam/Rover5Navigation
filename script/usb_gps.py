#!/usr/bin/env python
import roslib
from std_msgs.msg import String
import rospy
import serial

roslib.load_manifest('rover_5_navigation')

if __name__ == '__main__':
    # Publishing and Subscriptions
    logger = rospy.Publisher('logger', String, queue_size=10)
    
    rospy.init_node('usb_gps')

    serial_conn = serial.Serial('/dev/ttyUSB0', 4800, timeout = 5)

    while 1:
        line = serial_conn.readline()
        cs_line = line.split(',')

        if cs_line[0] == "$GPGGA":
            time = cs_line[1]
            lat = cs_line[2]
            latDir = cs_line[3]
            lon = cs_line[4]
            lonDir = cs_line[5]
            fix_quality = cs_line[6]
            sat_num = cs_line[7]
            horizontal_pos_dilution = cs_line[8]
            altitude = cs_line[9]
            alt_units = cs_line[10]
            geoid_height = cs_line[11]
            geoid_height_units = cs_line[12]

            if fix_quality == '0':
                log_string = 'No GPS signal - %s' % rospy.get_time()
                rospy.loginfo(log_string)
                logger.publish(log_string)

            print line
