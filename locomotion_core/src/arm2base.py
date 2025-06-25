#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist


global l_x, angular, max_du_pwr, effort_0
effort_0 = 0.0

max_du_pwr = 100;   # -1000 to +1000

def joint_state_callback(data):
    global effort_0
    effort_0 = data.effort[5]


def main():
    global l_x, angular, effort_0
    pub_frank_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('arm2base', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.Subscriber("/open_manipulator_p/joint_states", JointState, joint_state_callback)
        #rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
        
        print(effort_0)

        frank_cmd_msg = Twist()
        frank_cmd_msg.linear.x = effort_0/1000.0

        pub_frank_cmd.publish(frank_cmd_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
