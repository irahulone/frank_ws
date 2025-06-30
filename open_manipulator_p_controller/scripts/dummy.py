#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL


class CartesianForceToBaseController:
    def __init__(self):
        rospy.init_node("effort_to_base_with_jacobian")

        # Parameters
        self.base_link = rospy.get_param("~base_link", "base_link")
        self.ee_link = rospy.get_param("~ee_link", "gripper")
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.kx = rospy.get_param("~kx", 0.005)
        self.ky = rospy.get_param("~ky", 0.005)
        self.alpha = rospy.get_param("~alpha", 0.1)
        self.deadzone = rospy.get_param("~deadzone", 0.5)
        self.max_speed = rospy.get_param("~max_speed", 0.3)

        # Build KDL chain
        self.robot = URDF.from_parameter_server()
        ok, self.tree = treeFromUrdfModel(self.robot)
        if not ok:
            rospy.logerr("Failed to parse URDF into KDL tree.")
            return

        self.chain = self.tree.getChain(self.base_link, self.ee_link)
        self.jac_solver = PyKDL.ChainJntToJacSolver(self.chain)

        self.filtered_force = np.zeros(6)

        rospy.Subscriber("/open_manipulator_p/joint_states", JointState, self.joint_states_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.loginfo("Jacobian-based effort-to-base controller started.")
        rospy.spin()

    def joint_states_callback(self, msg):
        # Map joint name -> index
        joint_map = {name: i for i, name in enumerate(msg.name)}

        # Prepare joint position and effort arrays
        q = PyKDL.JntArray(len(self.joint_names))
        tau = np.zeros(len(self.joint_names))

        for i, name in enumerate(self.joint_names):
            if name in joint_map:
                idx = joint_map[name]
                q[i] = msg.position[idx]
                tau[i] = msg.effort[idx]
            else:
                rospy.logwarn("Missing joint: %s", name)
                return

        # Compute Jacobian
        J_kdl = PyKDL.Jacobian(len(self.joint_names))
        self.jac_solver.JntToJac(q, J_kdl)

        # Convert Jacobian to NumPy
        J = np.zeros((6, len(self.joint_names)))
        for i in range(6):
            for j in range(len(self.joint_names)):
                J[i, j] = J_kdl[i, j]

        # Compute Cartesian force
        try:
            F_ee = np.linalg.pinv(J.T).dot(tau)
        except np.linalg.LinAlgError:
            rospy.logwarn("Singular Jacobian, skipping this step")
            return

        # Low-pass filter
        self.filtered_force = self.alpha * F_ee + (1 - self.alpha) * self.filtered_force

        # Apply deadzone
        fx, fy = self.filtered_force[0], self.filtered_force[1]
        if abs(fx) < self.deadzone:
            fx = 0.0
        if abs(fy) < self.deadzone:
            fy = 0.0

        # Map to base motion
        cmd = Twist()
        cmd.linear.x = np.clip(self.kx * fx, -self.max_speed, self.max_speed)
        cmd.linear.y = np.clip(self.ky * fy, -self.max_speed, self.max_speed)
        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    try:
        CartesianForceToBaseController()
    except rospy.ROSInterruptException:
        pass
