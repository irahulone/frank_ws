// gravity_cancellation.h
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>


#include <memory>
#include <string>
#include <vector>

// Forward decl to keep header light; full include is in the .cpp
namespace KDL { class ChainDynParam; }

class GravityProbe {
public:
  GravityProbe(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  // Implementation is in the .cpp
  void computeAndPrint(const std::vector<double>& q_vec);

  // Callbacks/helpers
  void jointCb(const sensor_msgs::JointState::ConstPtr& msg);
  void timerCb(const ros::TimerEvent&);
  static bool mapJointStateToChain(const sensor_msgs::JointState& js,
                                   const std::vector<std::string>& chain_order,
                                   std::vector<double>* q_out);

  // ROS / params
  ros::NodeHandle nh_, pnh_;
  std::string urdf_param_, base_link_, tip_link_;
  double gx_{0.0}, gy_{0.0}, gz_{-9.81};

  // KDL
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
  std::vector<std::string> chain_joint_names_;
  KDL::JntArray q_, g_;

  // Mode / I/O
  bool cli_mode_{false};
  std::vector<double> q_cli_;
  ros::Subscriber sub_;
  ros::Timer timer_;
  sensor_msgs::JointState last_js_;
  bool have_js_{false};

  // Motor conversion parameters
  std::vector<double> k_t_eff_;        // Nm/A
  std::vector<double> a_per_tick_;     // A per tick
  std::vector<int16_t> max_abs_ticks_; // clamp (ticks)

  // --- in class GravityProbe (private) ---
  ros::Publisher current_pub_;
  ros::Publisher torque_pub_;             // optional for introspection
  bool enable_ = true;                    // param: enable/disable output
  double scale_ = 1.0;                    // param: scale the feedforward (0..1)

  // topic names (params)
  std::string current_topic_ = "goal_currents_ticks";
  std::string torque_topic_  = "gravity_torque";

};
