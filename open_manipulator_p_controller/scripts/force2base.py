#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped, Twist
import numpy as np

class ForceToBaseController:
    def __init__(self):
        rospy.init_node("force_to_base_controller")

        # Params
        self.kx = rospy.get_param("~kx", 0.001)
        self.ky = rospy.get_param("~ky", 0.001)
        self.deadzone = rospy.get_param("~deadzone", 1.0)  # N
        self.alpha = rospy.get_param("~alpha", 0.1)        # filter gain
        self.max_speed = rospy.get_param("~max_speed", 0.2)  # m/s

        self.bias_set = False
        self.bias = np.zeros(3)
        self.filtered_force = np.zeros(3)

        rospy.Subscriber("/wrench", WrenchStamped, self.wrench_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.loginfo("Force to base controller initialized. Waiting for force bias to be set...")
        rospy.spin()

    def wrench_callback(self, msg):
        raw_force = np.array([msg.wrench.force.x,
                              msg.wrench.force.y,
                              msg.wrench.force.z])

        if not self.bias_set:
            self.bias = raw_force
            self.bias_set = True
            rospy.loginfo(f"Force bias set to: {self.bias}")
            return

        # Subtract bias
        corrected_force = raw_force - self.bias

        # Low-pass filter
        self.filtered_force = self.alpha * corrected_force + (1 - self.alpha) * self.filtered_force

        # Apply deadzone
        fx, fy = self.filtered_force[0], self.filtered_force[1]
        if abs(fx) < self.deadzone:
            fx = 0.0
        if abs(fy) < self.deadzone:
            fy = 0.0

        # Map force to base velocity
        cmd = Twist()
        cmd.linear.x = np.clip(self.kx * fx, -self.max_speed, self.max_speed)
        cmd.linear.y = np.clip(self.ky * fy, -self.max_speed, self.max_speed)

        self.cmd_pub.publish(cmd)

