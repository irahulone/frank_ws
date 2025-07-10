#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from open_manipulator_msgs.msg import KinematicsPose
from open_manipulator_msgs.srv import SetKinematicsPose
import numpy as np

class RestoringAdmittanceController:
    def __init__(self):
        rospy.init_node('restoring_admittance_controller')

        # Control parameters
        self.k_spring = 15.0     # N/m, virtual spring stiffness
        self.damping = 20.0      # Ns/m, damping coefficient
        self.dt = 0.05           # seconds (20 Hz)

        # Storage
        self.external_force = np.zeros(3)
        self.current_pos = None
        self.rest_pos = None
        self.max_distance = 0.01  # meters, max allowed displacement from rest pose


        # ROS setup
        rospy.Subscriber("/task_wrench_filtered", WrenchStamped, self.wrench_callback)
        rospy.Subscriber("/gripper/kinematics_pose", KinematicsPose, self.pose_callback)

        rospy.wait_for_service('/goal_task_space_path_from_present_position_only')
        self.pose_client = rospy.ServiceProxy(
            '/goal_task_space_path_from_present_position_only', SetKinematicsPose)

        self.rate = rospy.Rate(1.0 / self.dt)

    def wrench_callback(self, msg):
        self.external_force[0] = msg.wrench.force.x
        self.external_force[1] = msg.wrench.force.y
        self.external_force[2] = msg.wrench.force.z

    def pose_callback(self, msg):
        pos = msg.pose.position
        self.current_pos = np.array([pos.x, pos.y, pos.z])
        # Store rest pose only once
        if self.rest_pos is None:
            self.rest_pos = np.array([pos.x, pos.y, pos.z])

    def run(self):
        while not rospy.is_shutdown():
            if self.current_pos is None or self.rest_pos is None:
                self.rate.sleep()
                continue

            # Compute restoring force
            restoring_force = -self.k_spring * (self.current_pos - self.rest_pos)

            # Net force = external + restoring
            net_force = self.external_force + restoring_force

            # Velocity from admittance model
            velocity = net_force / self.damping

            # New position
            new_pos = self.current_pos + velocity * self.dt

            # Calculate displacement from rest
            displacement = new_pos - self.rest_pos
            dist_norm = np.linalg.norm(displacement)

            # If displacement too large, scale it back
            if dist_norm > self.max_distance:
                displacement = displacement / dist_norm * self.max_distance
                new_pos = self.rest_pos + displacement

            # Create and send service request
            try:
                from open_manipulator_msgs.srv import SetKinematicsPoseRequest
                req = SetKinematicsPoseRequest()
                req.planning_group = "gripper"
                req.kinematics_pose.pose.position.x = new_pos[0]
                req.kinematics_pose.pose.position.y = new_pos[1]
                req.kinematics_pose.pose.position.z = new_pos[2]
                req.kinematics_pose.pose.orientation.w = 1.0  # neutral orientation
                req.path_time = self.dt

                self.pose_client.call(req)
                rospy.logerr("going to: {}".format(new_pos[0]))

            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))


            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RestoringAdmittanceController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
