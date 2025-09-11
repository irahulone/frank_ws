#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <memory>
#include <string>
#include <vector>

// Keep header light: forward-declare KDL solvers; .cpp includes full headers.
namespace KDL {
  class ChainDynParam;
  class ChainFkSolverPos_recursive;
  class ChainJntToJacSolver;
}

class ImpedanceProbe {
public:
  ImpedanceProbe(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
 
  KDL::Frame x_des{ KDL::Vector(0.5, 0.0, 0.2) };   // ✅ use braces

  // Core compute (now takes q, dq)
  void computeAndPublish(const std::vector<double>& q_vec,
                         const std::vector<double>& dq_vec);

  // Callbacks/helpers
  void jointCb(const sensor_msgs::JointState::ConstPtr& msg);
  void xdCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void vdCb(const geometry_msgs::Twist::ConstPtr& msg);
  void timerCb(const ros::TimerEvent&);

  static bool mapJointStateToChain(const sensor_msgs::JointState& js,
                                   const std::vector<std::string>& chain_order,
                                   std::vector<double>* q_out,
                                   std::vector<double>* dq_out);

  // Small orientation-error helper (small-angle, world frame)
  static KDL::Vector rotVecError(const KDL::Rotation& R_now,
                                 const KDL::Rotation& R_des);

  // ================= ROS / params =================
  ros::NodeHandle nh_, pnh_;
  std::string urdf_param_, base_link_, tip_link_;
  double gx_{0.0}, gy_{0.0}, gz_{-9.81};
  bool   enable_{true};
  double scale_{1.0};
 
  // Topics
  std::string current_topic_{"goal_currents_ticks"};
  std::string torque_topic_{"impedance_torque"};

  // IO
  ros::Subscriber sub_js_;
  ros::Subscriber sub_xd_;
  ros::Subscriber sub_vd_;
  ros::Publisher  current_pub_;
  ros::Publisher  torque_pub_;
  ros::Timer      timer_;

  // Desired task state
  geometry_msgs::PoseStamped xd_msg_;
  geometry_msgs::Twist       vd_msg_;
  bool have_js_{false}, have_xd_{false}, have_vd_{false}, have_x_hold_{true};
  sensor_msgs::JointState last_js_;

  // ================= KDL model & state ==============
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam>            dyn_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::unique_ptr<KDL::ChainJntToJacSolver>        jac_solve_;
  std::vector<std::string> chain_joint_names_;
  KDL::Frame x_hold_;   // latched equilibrium pose

  KDL::JntArray q_, dq_, g_;   // joint pos/vel/gravity
  KDL::Jacobian J_;
  KDL::Frame    x_now_;

  // ================= Impedance gains =================
  // Cartesian translational [N/m] and rotational [N·m/rad], plus damping
  KDL::Vector K_xyz_{15,15,15};
  KDL::Vector D_xyz_{ 1,  1,  1};
  KDL::Vector K_rot_{  0,  0,  0};
  KDL::Vector D_rot_{ 0, 0, 0};

  // ================= Motor conversion =================
  // Your existing current/torque mapping (per-joint)
  std::vector<double> k_t_eff_;        // Nm/A
  std::vector<double> a_per_tick_;     // A / tick
  std::vector<int>    max_abs_ticks_;  // clamp (ticks)

  // ================= CLI test mode ====================
  bool cli_mode_{false};
  std::vector<double> q_cli_;

  // --- members in your class (ImpedanceProbe / Controller) ---
  int n_joints_{6};

  Eigen::VectorXd q_hold_;       // latched equilibrium posture
  Eigen::VectorXd dq_filt_;      // filtered joint velocities
  Eigen::VectorXd Kq_, Dq_;      // Nm/rad, Nms/rad
  Eigen::VectorXd tau_prev_;     // last commanded torque (for rate limit)
  Eigen::VectorXd tau_limits_;   // per-joint |Nm| limits (optional)

  bool have_q_hold_{false};
  double vel_cutoff_hz_{15.0};   // LPF cutoff for dq
  double max_tau_rate_{50.0};    // Nm/s, rate limiter (optional)

  double dt;

};