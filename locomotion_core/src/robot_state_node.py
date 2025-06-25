#! /usr/bin/env python

import rospy
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import String

global rover_modeC, case_number, timeout, joy_lx, joy_az, ply_lx, ply_az, rover_enable
rover_modeC = "NEU_M"
case_number = 0 
timeout = 0
joy_lx = 0
joy_az = 0
ply_lx = 0
ply_az = 0
rover_enable = False


def callback_robot_cmd_vel(data):
    global ply_lx, ply_az
    ply_lx = data.linear.x
    ply_az = data.angular.z
    
def callback_cmd_vel(data):
    global tea_lx, tea_az
    tea_lx = data.linear.x
    tea_az = data.angular.z
    
def callback_joy(data):
    
    global rover_modeC, case_number, timeout, joy_lx, joy_az, ply_lx, ply_az, rover_enable
    deadman_button_state = data.buttons[4]
    if rover_modeC == "NEU_M":
         deadman_button_state = 0
    
    enable = Bool()
    enable.data = False
    if (deadman_button_state == 1):
        enable.data = True
    rover_enable = enable.data

    pub_rover_enable = rospy.Publisher('r1/enable', Bool, queue_size=1)
    pub_rover_enable.publish(enable)
    
    ################# toggle robot modes by pressing buttton A
    togg_button = data.buttons[0]
    
    if togg_button == 1:
        time.sleep(0.1)
        case_number += 0.334
        
        if case_number > 3:
            case_number = 0
    
    if case_number >= 0 and case_number < 1:
        rover_modeC = "NEU_M"
    elif case_number >= 1 and case_number < 2:
        rover_modeC = "TEA_M"
    elif case_number >= 2 and case_number < 3:
        rover_modeC = "PLY_M"
        
    robot_mode_pub = rospy.Publisher('r1/modeC', String, queue_size=10)
    robot_mode_pub.publish(rover_modeC)
    ################# end of toggle modes
    
    core_rover_vel = Twist()
    core_rover_vel_pub = rospy.Publisher('r1/core_cmd_vel', Twist, queue_size=10)
    
    if rover_modeC == "TEA_M":
        if rover_enable == False:
            core_rover_vel.linear.x = 0;
            core_rover_vel.angular.z = 0;
        elif rover_enable == True:
            core_rover_vel.linear.x = tea_lx;
            core_rover_vel.angular.z = tea_az;
        
    elif rover_modeC == "PLY_M":
        if rover_enable == False:
            core_rover_vel.linear.x = 0;
            core_rover_vel.angular.z = 0;
        elif rover_enable == True:
            core_rover_vel.linear.x = ply_lx;
            core_rover_vel.angular.z = ply_az;
    else:
        core_rover_vel.linear.x = 0;
        core_rover_vel.angular.z = 0;
        
    core_rover_vel_pub.publish(core_rover_vel)
    
    
    
    

def start():
    
    rospy.init_node('robot_state_node')    
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback_joy)
    rospy.Subscriber("cmd_vel", Twist, callback_cmd_vel)
    rospy.Subscriber("r1/cmd_vel", Twist, callback_robot_cmd_vel)
    # starts the node
    
    rospy.spin()

if __name__ == '__main__':
    start()


