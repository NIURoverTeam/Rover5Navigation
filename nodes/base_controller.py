#!/usr/bin/env python
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rospy
import serial
import struct
from time import sleep
import tf
roslib.load_manifest('rover_5_navigation')

ser = serial.Serial('/dev/ttyACM0', 9600) 

# Function called when Twist messages are published
def navigation(data):
    logger.publish("Recieved")

    linVel = data.linear.x * 255 / 2 * 8
    rotVel = data.angular.z * 255 / 2 * 66
    # Combine, divide by radius, and scale to fit into an integer byte
    leftPower = round((linVel + rotVel))
    if leftPower >= 255:
        leftPower = 255
    if leftPower <= -255:
        leftPower = -255
    rightPower = round((linVel - rotVel))
    if rightPower >= 255:
        rightPower = 255
    if rightPower <= -255:
        rightPower = -255

    if leftPower >= 0:
        if rightPower >= 0:
           ser.write(struct.pack('>BBBB',76, int(leftPower), 82, int(rightPower)))
        else:
            ser.write(struct.pack('>BBBB',108, int(leftPower), 82, int(-rightPower)))
    else:
        if rightPower <= 0:
            ser.write(struct.pack('>BBBB',108, int(-leftPower),114, int(-rightPower)))
        else:
            ser.write(struct.pack('>BBBB',76, int(-leftPower), 114, int(rightPower)))


    # if rotational_power >= 0:
    #    # Sends 'Lr'
    #     ser.write(struct.pack('>BBBB',108, int(left_power), 82, int(rotational_power)))
    # else:
    #     # Sends 'lR'
    #     ser.write(struct.pack('>BBBB',76, int(-left_power), 114, int(-rotational_power)))

    # if forward_power >= 0:
    #     # Sends 'LR'
    #     ser.write(struct.pack('>BBBB',76, int(left_power), 82, int(forward_power)))
    # else:
    #     # Sends 'lr'
    #     ser.write(struct.pack('>BBBB',108, int(-left_power),114, int(-forward_power)))


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