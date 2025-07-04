#!/usr/bin/env python
from geometry_msgs.msg import Twist
import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np

class TaskWrenchFilter:
    def __init__(self):
        rospy.init_node("task_wrench_filter")

        # Parameters
        self.alpha = rospy.get_param("~alpha", 0.1)
        self.force_deadzone = rospy.get_param("~force_deadzone", 1.0)     # in Newtons
        self.torque_deadzone = rospy.get_param("~torque_deadzone", 0.1)  # in Nm
        self.frame_id = rospy.get_param("~frame_id", "gripper")

        self.bias_set = False
        self.bias_force = np.zeros(3)
        self.bias_torque = np.zeros(3)
        self.filtered_force = np.zeros(3)
        self.filtered_torque = np.zeros(3)
        self.idle_force_threshold = rospy.get_param("~idle_force_threshold", 5.0)  # N
        self.reset_when_idle = rospy.get_param("~reset_when_idle", True)
        self.was_idle = False  # Add in __init__
        self.cmd_vel = Twist()  # To store last command

        # Publisher
        self.pub = rospy.Publisher('/task_wrench_filtered', WrenchStamped, queue_size=10)

        # Subscriber
        rospy.Subscriber("/task_wrench", WrenchStamped, self.wrench_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.loginfo("Task Wrench Filter initialized. Waiting for first message to set bias...")

    def apply_deadzone(self, vector, threshold):
        return np.where(np.abs(vector) < threshold, 0.0, vector)
    
    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def is_cmd_vel_idle(self, threshold=0.1):
        lin = self.cmd_vel.linear
        ang = self.cmd_vel.angular
        return (abs(lin.x) < threshold and abs(lin.y) < threshold and abs(lin.z) < threshold and
                abs(ang.x) < threshold and abs(ang.y) < threshold and abs(ang.z) < threshold)


    def wrench_callback(self, msg):
        force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        torque = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])

        if not self.bias_set:
            self.bias_force = force
            self.bias_torque = torque
            self.bias_set = True
            rospy.loginfo("Wrench bias set.")
            return

        # Remove bias
        force_corrected = force - self.bias_force
        torque_corrected = torque - self.bias_torque

        # Apply deadzone
        force_corrected = self.apply_deadzone(force_corrected, self.force_deadzone)
        torque_corrected = self.apply_deadzone(torque_corrected, self.torque_deadzone)

        # Apply low-pass filter
        self.filtered_force = self.alpha * force_corrected + (1 - self.alpha) * self.filtered_force
        self.filtered_torque = self.alpha * torque_corrected + (1 - self.alpha) * self.filtered_torque

        # Auto-reset filtered wrench if "idle"
        is_wrench_idle = np.linalg.norm(self.filtered_force) < self.idle_force_threshold
        is_base_idle = self.is_cmd_vel_idle()


        if self.reset_when_idle and is_wrench_idle and is_base_idle:
            if not self.was_idle:
                rospy.loginfo("Robot appears idle — resetting wrench.")
                self.was_idle = True
            self.filtered_force = np.zeros(3)
            self.filtered_torque = np.zeros(3)
        else:
            self.was_idle = False

        # Prepare message
        filtered_msg = WrenchStamped()
        filtered_msg.header = msg.header
        filtered_msg.header.frame_id = self.frame_id

        filtered_msg.wrench.force.x = self.filtered_force[0]
        filtered_msg.wrench.force.y = self.filtered_force[1]
        filtered_msg.wrench.force.z = self.filtered_force[2]

        filtered_msg.wrench.torque.x = self.filtered_torque[0]
        filtered_msg.wrench.torque.y = self.filtered_torque[1]
        filtered_msg.wrench.torque.z = self.filtered_torque[2]

        self.pub.publish(filtered_msg)

if __name__ == "__main__":
    TaskWrenchFilter()
    rospy.spin()
