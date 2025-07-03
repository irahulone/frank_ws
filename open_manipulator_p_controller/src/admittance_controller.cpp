#include "open_manipulator_p_controller/admittance_controller.h"
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <Eigen/Dense>
using namespace robotis_manipulator;

namespace open_manipulator_p_controllers
{

AdmittanceController::AdmittanceController(ros::NodeHandle& nh, const std::string& usb_port, const std::string& baud_rate)
: nh_(nh),priv_node_handle_("~"),
  with_gripper_(false),
  usb_port_(usb_port),
  baud_rate_(baud_rate)
{
  init();
}


bool AdmittanceController::init()
{
  control_period_       = priv_node_handle_.param<double>("control_period", 0.010f);
  using_platform_       = priv_node_handle_.param<bool>("using_platform", false);
  with_gripper_         = priv_node_handle_.param<bool>("with_gripper", false);
  // Subscriber: joint_states
  joint_state_sub_ = nh_.subscribe("/joint_states", 10, &AdmittanceController::jointStateCb, this);

  // Publisher: End-effector wrench
  tau_des_pub_ = nh_.advertise<geometry_msgs::Wrench>("/ee_wrench", 10);

  ROS_INFO("Admittance controller initialized");

  open_manipulator_.initOpenManipulator(using_platform_, usb_port_, baud_rate_, control_period_, with_gripper_);

  // You may need to load or define manipulator kinematics
  // For example, you might set up the kinematic chain here
  // Or load the robot description and call manipulator_.addComponent(), etc.
  // kinematics::SolverUsingCRAndJacobian kinematics_solver_;


  return true;
}

void AdmittanceController::jointStateCb(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (msg->effort.empty() || msg->position.empty())
  {
    ROS_WARN_THROTTLE(1.0, "JointState message missing effort or position.");
    return;
  }

  // Step 1: Update manipulator joint positions (important for Jacobian computation)
  std::vector<double> joint_angles = msg->position;
  std::vector<robotis_manipulator::JointValue> joint_values;

  for (const auto& angle : joint_angles)
  {
    robotis_manipulator::JointValue jv;
    jv.position = angle;
    joint_values.push_back(jv);
  }

  // manipulator_.setAllJointValue(joint_values);

  // Step 2: Compute Jacobian at the tool frame
  Eigen::MatrixXd J = open_manipulator_.jacobian("gripper");

  ROS_INFO_STREAM("Some message: " << J);
  // Step 3: Get joint effort as Eigen vector
  Eigen::VectorXd tau(msg->effort.size());
  for (size_t i = 0; i < msg->effort.size(); ++i)
  {
    tau[i] = msg->effort[i];
  }

  // Step 4: Compute end-effector wrench: wrench = J * tau
  Eigen::VectorXd wrench = J * tau;

  // Step 5: Publish the result as geometry_msgs::Wrench (first 3 force, next 3 torque)
  geometry_msgs::Wrench wrench_msg;
  if (wrench.size() >= 6)
  {
    wrench_msg.force.x = wrench[0];
    wrench_msg.force.y = wrench[1];
    wrench_msg.force.z = wrench[2];
    wrench_msg.torque.x = wrench[3];
    wrench_msg.torque.y = wrench[4];
    wrench_msg.torque.z = wrench[5];
  }
  else
  {
    ROS_WARN_THROTTLE(1.0, "Jacobian result has less than 6 rows. Cannot form full wrench.");
    return;
  }

  tau_des_pub_.publish(wrench_msg);
}

Eigen::VectorXd AdmittanceController::computeEndEffectorWrench(const Eigen::VectorXd& joint_effort)
{
  Eigen::MatrixXd J = open_manipulator_.jacobian("gripper");
  return J * joint_effort;
}

} // namespace open_manipulator_p_controllers


int main(int argc, char **argv)
{
  // init
  ros::init(argc, argv, "admittance_controller");
  // ros::NodeHandle node_handle("");
  ros::NodeHandle nh;

  std::string usb_port = "/dev/ttyUSB0";
  std::string baud_rate = "1000000";

  if (argc == 3)
  {
    usb_port = argv[1];
    baud_rate = argv[2];
    printf("port_name and baud_rate are set to %s, %s \n", usb_port.c_str(), baud_rate.c_str());
  }
  else
  {
    log::error("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 1;
  }
  open_manipulator_p_controllers::AdmittanceController controller(nh, usb_port, baud_rate);

  ros::spin();
  return 0;
}