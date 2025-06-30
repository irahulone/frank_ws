#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np

class EffortToBaseController:
    def __init__(self):
        rospy.init_node("effort_to_base_controller")

        # Parameters
        self.kx = rospy.get_param("~kx", 0.01)
        self.ky = rospy.get_param("~ky", 0.01)
        self.alpha = rospy.get_param("~alpha", 0.1)
        self.deadzone = rospy.get_param("~deadzone", 0.2)
        self.max_speed = rospy.get_param("~max_speed", 0.2)
        self.joint_names = rospy.get_param("~effort_joints", ["joint1", "joint2"])  # Assume planar 2D

        self.bias_set = False
        self.bias = np.zeros(len(self.joint_names))
        self.filtered_effort = np.zeros(len(self.joint_names))

        # Publisher to publish effort-only JointState
        self.pub = rospy.Publisher('/open_manipulator_p/effort_only', JointState, queue_size=10)

        # self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        rospy.loginfo("Effort-to-base controller initialized. Waiting for bias calibration...")

    def joint_states_callback(self, msg):
        # Get effort values for relevant joints
        effort_dict = dict(zip(msg.name, msg.effort))
        current_efforts = np.array([effort_dict.get(j, 0.0) for j in self.joint_names])
        # current_efforts = msg.effort


        if not self.bias_set:
            self.bias = current_efforts
            self.bias_set = True
            rospy.loginfo("Effort bias set to: {}".format(self.bias))
            return

        # Remove gravity bias
        effort_corrected = current_efforts - self.bias

        # Low-pass filter
        self.filtered_effort = self.alpha * effort_corrected + (1 - self.alpha) * self.filtered_effort

        # Deadzone
        # eff_x, eff_y = self.filtered_effort
        # if abs(eff_x) < self.deadzone:
        #     eff_x = 0.0
        # if abs(eff_y) < self.deadzone:
        #     eff_y = 0.0

        # Create a new JointState message to publish only effort info
        effort_msg = JointState()
        
        # Copy the header to keep timestamps etc.
        effort_msg.header = msg.header
        
        # Copy the joint names
        effort_msg.name = msg.name
        
        # We only care about effort, so set position and velocity to empty lists
        effort_msg.position = []
        effort_msg.velocity = []
        
        # Copy effort data
        effort_msg.effort = self.filtered_effort

        # Publish the effort message
        self.pub.publish(effort_msg)


        # Map effort to velocity command
        # cmd = Twist()
        # cmd.linear.x = np.clip(self.kx * eff_x, -self.max_speed, self.max_speed)
        # cmd.linear.y = np.clip(self.ky * eff_y, -self.max_speed, self.max_speed)

        # self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    EffortToBaseController()
    rospy.spin()
