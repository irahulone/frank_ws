#ifndef ADMITTANCE_CONTROLLER_H_
#define ADMITTANCE_CONTROLLER_H_

#include <vector>
#include <string>
#include <map>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "open_manipulator_p_libs/open_manipulator_p.h"
#include "open_manipulator_p_libs/kinematics.h"          // ✅ spelling!

namespace open_manipulator_p_controllers
{

class AdmittanceController
{
public:
  explicit AdmittanceController(ros::NodeHandle& nh);
  ~AdmittanceController() = default;

  /** Initialise publishers, subscribers, parameters, etc. */
  bool init();

private:
  /* ---------- ROS plumbing ---------- */
  ros::NodeHandle           nh_;
  ros::Subscriber           joint_state_sub_;   // listens to /joint_states
  ros::Publisher            tau_des_pub_;       // publishes desired effort or trajectory

  /* ---------- Manipulator / kinematics ---------- */
  open_manipulator_p::Manipulator manipulator_;
  kinematics::SolverUsingCRAndJacobian kinematics_solver_;

  /* ---------- Callbacks ---------- */
  void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg);

  /* ---------- Helpers ---------- */
  /** Computes Jacobian × τ  → end‑effector wrench (or whatever you need). */
  Eigen::VectorXd computeEndEffectorWrench(const Eigen::VectorXd& joint_effort);
};

}  // namespace open_manipulator_p_controllers

#endif  // ADMITTANCE_CONTROLLER_H_
