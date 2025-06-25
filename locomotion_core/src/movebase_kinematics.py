#!/usr/bin/env python


import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

global linear, angular, max_du_pwr
linear = 0.0; angular = 0.0

max_du_pwr = 100;   # -1000 to +1000

def cmd_vel_callback(data):
    global linear, angular
    linear = data.linear.x
    angular = data.angular.z

def main():
    global linear, angular
    pub_vel_m1 = rospy.Publisher('vel_motor1', Int16, queue_size=10)
    pub_vel_m2 = rospy.Publisher('vel_motor2', Int16, queue_size=10)
    pub_vel_m3 = rospy.Publisher('vel_motor3', Int16, queue_size=10)
    pub_vel_m4 = rospy.Publisher('vel_motor4', Int16, queue_size=10)

    rospy.init_node('kin_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
                
        m1_vel = -int( 50*linear - 40*angular)
        m2_vel = int( 50*linear + 40*angular)
        m3_vel = int( 50*linear - 40*angular)
        m4_vel = -int( 50*linear + 40*angular)

        pub_vel_m1.publish(m1_vel)
        pub_vel_m2.publish(m2_vel)
        pub_vel_m3.publish(m3_vel)
        pub_vel_m4.publish(m4_vel)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
