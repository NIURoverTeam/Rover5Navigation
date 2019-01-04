#!/usr/bin/env python
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rospy
import serial
import struct
from time import sleep
import tf
roslib.load_manifest('rover_core_os')

# Function called when Twist messages are published
def navigation(data):
    logger.publish("Recieved!")

    


if __name__ == '__main__':
    #Prep all the topics we want to publish/subscribe to
    logger = rospy.Publisher('logger', String, queue_size=10)

    rospy.Subscriber("cmd_vel", Twist, navigation)

    # Start the driver node
    rospy.init_node('base_controller')

    # Start node logging
    log_string = 'Starting base_controller....%s' % rospy.get_time()
    rospy.loginfo(log_string)
    logger.publish(log_string)

rospy.spin()