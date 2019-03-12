#!/usr/bin/env python
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import rospy
import struct
from time import sleep
import tf
import smbus

roslib.load_manifest('rover_5_navigation')

bus = smbus.SMBus(0)
dev_adr = 0x0a
control_char = 0x24

def controller_input(data):
    # Check if the back button has been pressed, and if it has, shutdown
    if data.buttons[6] == 1:
        logger.publish("Back Buttton Pressed, shutting down...")
        rospy.signal_shutdown("Back Button Pressed!")

    # Extract relevant data into useful local copies
    buttons = data.buttons # Array of int32
    axes = data.axes # Arrary of float32, each from -1.0 to 1.0

    # Combine, divide by radius, and scale to fit into an integer byte
    leftPower = round(axes[1] * 50)
    rightPower = round(axes[4] * 50)
    leftDir = 0x4C if leftPower >= 0 else 0x6C
    rightDir = 0x52 if rightPower >= 0 else 0x72


    bus.write_i2c_block_data(dev_adr,control_char,[leftDir,int(abs(leftPower)),rightDir,int(abs(rightPower))])

if __name__ == '__main__':
    #Prep all the topics we want to publish/subscribe to
    logger = rospy.Publisher('logger', String, queue_size=10)

    rospy.Subscriber("joy", Joy, controller_input)

    # Start the driver node
    rospy.init_node('base_controller_i2c')

    # Start node logging
    log_string = 'Starting base_controller_i2c....%s' % rospy.get_time()
    rospy.loginfo(log_string)
    logger.publish(log_string)
    

rospy.spin()
