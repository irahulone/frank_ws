#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

from pyroboteq_mc import RoboteQ_MC
roboteq1 = RoboteQ_MC('/dev/ttyACM0')
roboteq2 = RoboteQ_MC('/dev/ttyACM1')

max_pwr = 500

def callback_m1(data):
    x = data.data
    if x > max_pwr:
        x = max_pwr
    elif x < -max_pwr:
        x = -max_pwr
    val = x
    roboteq1.channel_1(val)

def callback_m2(data):
    x = data.data
    if x > max_pwr:
        x = max_pwr
    elif x < -max_pwr:
        x = -max_pwr
    val = x
    roboteq1.channel_2(val)

def callback_m3(data):
    x = data.data
    if x > max_pwr:
        x = max_pwr
    elif x < -max_pwr:
        x = -max_pwr
    val = x
    roboteq2.channel_2(val)

def callback_m4(data):
    x = data.data
    if x > max_pwr:
        x = max_pwr
    elif x < -max_pwr:
        x = -max_pwr
    val = x
    roboteq2.channel_1(val)


def main():

    rospy.init_node('base_motor_drivers', anonymous=True)
    rospy.Subscriber("vel_motor1", Int16, callback_m1)
    rospy.Subscriber("vel_motor2", Int16, callback_m2)
    rospy.Subscriber("vel_motor3", Int16, callback_m3)
    rospy.Subscriber("vel_motor4", Int16, callback_m4)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
