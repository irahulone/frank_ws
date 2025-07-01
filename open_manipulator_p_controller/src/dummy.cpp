#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <urdf/model.h>
#include <ros/ros.h>

bool computeJacobian(const std::string& urdf_param, const std::string& root_link,
                     const std::string& tip_link, const std::vector<double>& joint_positions)
{
  // Parse URDF model
  urdf::Model urdf_model;
  if (!urdf_model.initParam(urdf_param))
  {
    ROS_ERROR("Failed to parse URDF");
    return false;
  }

  // Generate KDL tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree))
  {
    ROS_ERROR("Failed to create KDL tree");
    return false;
  }

  // Extract chain
  KDL::Chain kdl_chain;
  if (!kdl_tree.getChain(root_link, tip_link, kdl_chain))
  {
    ROS_ERROR("Failed to extract KDL chain");
    return false;
  }

  // Create Jacobian solver
  KDL::ChainJntToJacSolver jac_solver(kdl_chain);

  // Populate joint positions
  KDL::JntArray q(kdl_chain.getNrOfJoints());
  for (size_t i = 0; i < joint_positions.size(); ++i)
    q(i) = joint_positions[i];

  // Compute Jacobian
  KDL::Jacobian jacobian(kdl_chain.getNrOfJoints());
  if (jac_solver.JntToJac(q, jacobian) < 0)
  {
    ROS_ERROR("Failed to compute Jacobian");
    return false;
  }

  // Print Jacobian
  std::cout << "Jacobian:\n" << jacobian.data << std::endl;

  return true;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "jacobian_example");
  ros::NodeHandle nh;

  std::vector<double> joint_positions = {0.0, 0.5, -0.3, 0.1}; // replace with your robot's DOF

  computeJacobian("robot_description", "base_link", "tool0", joint_positions);

  return 0;
}
