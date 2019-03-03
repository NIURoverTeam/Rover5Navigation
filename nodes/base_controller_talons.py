#!/usr/bin/env python
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import rospy
import serial
import struct
from time import sleep
import tf
roslib.load_manifest('rover_5_navigation')

ser = serial.Serial('/dev/ttyACM0', 115200)

# Function called when Twist messages are published
def navigation(data):
    logger.publish("Recieved")

    linVel = data.linear.x * 50
    rotVel = data.angular.z * 50
    # Combine, divide by radius, and scale to fit into an integer byte
    leftPower = round((linVel + rotVel))
    if leftPower >= 100:
        leftPower = 100
    if leftPower <= -100:
        leftPower = -100
    rightPower = round((linVel - rotVel))
    if rightPower >= 100:
        rightPower = 100
    if rightPower <= -100:
        rightPower = -100

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

def controller_input(data):
    # Check if the back button has been pressed, and if it has, shutdown
    if data.buttons[6] == 1:
        logger.publish("Back Buttton Pressed, shutting down...")
        rospy.signal_shutdown("Back Button Pressed!")

    # Extract relevant data into useful local copies
    buttons = data.buttons # Array of int32
    # 0     A
    # 1     B
    # 2     X
    # 3     Y
    # 4     LB
    # 5     RB
    # 6     Back
    # 7     Start
    # 8     Middle Button
    # 9     Left Stick Click
    # 10    Right Stick Click
    axes = data.axes # Arrary of float32, each from -1.0 to 1.0
    # 0     Left Thumbstick : 1:Left, -1:Right
    # 1     Left Thumbstick : 1:Up, -1:Down
    # 2     Left Trigger : -1:Pressed, 1:Released
    # 3     Right Thumbstick : 1:Left, -1:Right
    # 4     Right Thumbstick : 1:Up, -1:Down
    # 5     Right Trigger : -1:Pressed, 1:Released
    # 6     D-PAD : 1:Left, -1:Right
    # 7     D-PAD : 1:Up, -1:Down

    # Combine, divide by radius, and scale to fit into an integer byte
    leftPower = round(axes[1] * 100)
    rightPower = round(axes[4] * 100)

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


if __name__ == '__main__':
    #Prep all the topics we want to publish/subscribe to
    logger = rospy.Publisher('logger', String, queue_size=10)

    rospy.Subscriber("cmd_vel", Twist, navigation)
    rospy.Subscriber("joy", Joy, controller_input)

    # Start the driver node
    rospy.init_node('base_controller_talons')

    # Start node logging
    log_string = 'Starting base_controller_talons....%s' % rospy.get_time()
    rospy.loginfo(log_string)
    logger.publish(log_string)
    

rospy.spin()