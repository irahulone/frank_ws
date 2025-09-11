#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
#include <string>

#include "open_manipulator_p_controller/current_map.h"     // provides CurrentMap, torqueNmToGoalCurrentTick(...)
#include "open_manipulator_p_controller/gravity_cancellation.h"

// --- Small helpers ---
static bool parseCommaDoubles(const std::string& s, std::vector<double>* out) {
  out->clear();
  std::stringstream ss(s);
  for (std::string item; std::getline(ss, item, ','); ) {
    if (item.empty()) continue;
    try { out->push_back(std::stod(item)); } catch (...) { return false; }
  }
  return !out->empty();
}

GravityProbe::GravityProbe(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh) {
  // Parameters
  pnh_.param<std::string>("urdf_param", urdf_param_, "/robot_description");
  pnh_.param<std::string>("base_link",  base_link_,  "open_manipulator_origin");
  pnh_.param<std::string>("tip_link",   tip_link_,   "open_manipulator_p_tool");
  pnh_.param<double>("gx", gx_, 0.0);
  pnh_.param<double>("gy", gy_, 0.0);
  pnh_.param<double>("gz", gz_, -9.81);

  std::string q_cli;
  pnh_.param<std::string>("q", q_cli, "");
  if (!q_cli.empty()) {
    cli_mode_ = parseCommaDoubles(q_cli, &q_cli_);
    if (!cli_mode_) {
      ROS_ERROR("Failed to parse --q string. Expected comma-separated radians (e.g. --q 0,0.5,-0.3,0)");
      ros::shutdown();
      return;
    }
  }

  // Load URDF
  std::string urdf_xml;
  if (!nh_.getParam(urdf_param_, urdf_xml)) {
    ROS_ERROR_STREAM("URDF not found on param '" << urdf_param_ << "'");
    ros::shutdown(); return;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree)) {
    ROS_ERROR("Failed to parse URDF into KDL tree");
    ros::shutdown(); return;
  }

  pnh_.param("enable", enable_, true);
  pnh_.param("scale",  scale_,  1.0);
  pnh_.param<std::string>("current_topic", current_topic_, "goal_currents_ticks");
  pnh_.param<std::string>("torque_topic",  torque_topic_,  "gravity_torque");

  current_pub_ = nh_.advertise<std_msgs::Int16MultiArray>(current_topic_, 10);
  torque_pub_  = nh_.advertise<std_msgs::Float64MultiArray>(torque_topic_, 10); // optional

  if (!tree.getChain(base_link_, tip_link_, chain_)) {
    ROS_ERROR_STREAM("Failed to get KDL chain from '" << base_link_
                        << "' to '" << tip_link_ << "'");
    ros::shutdown(); return;
  }

  // Build joint name order
  chain_joint_names_.clear();
  for (unsigned i = 0; i < chain_.getNrOfSegments(); ++i) {
    const auto& seg = chain_.getSegment(i);
    const auto& j = seg.getJoint();
    if (j.getType() != KDL::Joint::None)
      chain_joint_names_.push_back(j.getName());
  }

  // KDL dynamics (gravity vector in base frame)
  dyn_.reset(new KDL::ChainDynParam(chain_, KDL::Vector(gx_, gy_, gz_)));
  q_.resize(chain_.getNrOfJoints());
  g_.resize(chain_.getNrOfJoints());

  // Initialize motor conversion arrays (replace with real values or param-load)
  const size_t N = chain_.getNrOfJoints();
  k_t_eff_.resize(N);
  a_per_tick_.resize(N);
  max_abs_ticks_.resize(N);

  // Map each joint to its actuator model
  enum class Model { PH54_200, PH54_100, PH42_020 };
  std::vector<Model> model_of_joint = {
  /* J1 */ Model::PH54_200,
  /* J2 */ Model::PH54_200,
  /* J3 */ Model::PH54_100,
  /* J4 */ Model::PH54_100,
  /* J5 */ Model::PH42_020,
  /* J6 */ Model::PH42_020,
  };

  // Per-joint limits you gave
  max_abs_ticks_ = {22740, 22740, 15900, 15900, 4500, 4500};

  // Fill conversion constants per model
  for (size_t i = 0; i < N; ++i) {
  switch (model_of_joint[i]) {
    case Model::PH54_200:
    // If you’re using PH54-200 on some joints:
    // a_per_tick = 0.001 A/LSB (1 mA) and k_t_eff ≈ 4.81 Nm/A (44.7 Nm / 9.3 A)
    a_per_tick_[i] = 0.001;
    k_t_eff_[i]    = 4.81;
    break;
    case Model::PH54_100:
    a_per_tick_[i] = 0.001;
    k_t_eff_[i]    = 4.60;  // 25.3 / 5.5
    break;
    case Model::PH42_020:
    a_per_tick_[i] = 0.001;
    k_t_eff_[i]    = 3.40;  // 5.1 / 1.5
    break;
  }
  }
  

  ROS_INFO_STREAM("GravityProbe ready. DOF=" << N
                  << " base='" << base_link_ << "' tip='" << tip_link_
                  << "' g=[" << gx_ << "," << gy_ << "," << gz_ << "]");

  if (cli_mode_) {
    if (q_cli_.size() != N) {
      ROS_ERROR_STREAM("CLI q has size " << q_cli_.size() << " but chain has " << N);
      ros::shutdown(); return;
    }
    computeAndPublish(q_cli_);
    ros::shutdown(); return;
  }

  // Live mode
  sub_   = nh_.subscribe("/joint_states", 1, &GravityProbe::jointCb, this);
  timer_ = nh_.createTimer(ros::Duration(0.1), &GravityProbe::timerCb, this); // 10 Hz
}


void GravityProbe::jointCb(const sensor_msgs::JointState::ConstPtr& msg) {
  last_js_ = *msg;
  have_js_ = true;
}

void GravityProbe::timerCb(const ros::TimerEvent&) {
  if (!have_js_) {
    ROS_WARN_THROTTLE(2.0, "Waiting for /joint_states...");
    return;
  }
  std::vector<double> q(chain_joint_names_.size(), 0.0);
  if (!mapJointStateToChain(last_js_, chain_joint_names_, &q)) {
    ROS_WARN_THROTTLE(1.0, "JointState missing some chain joints.");
    return;
  }
  // Compute and publish
  computeAndPublish(q);
}


void GravityProbe::computeAndPublish(const std::vector<double>& q_vec) {
  // Fill KDL array
  for (size_t i = 0; i < q_vec.size(); ++i) q_(i) = q_vec[i];

  std::ostringstream oss;
  for (size_t i = 0; i < q_vec.size(); ++i) {
    oss << q_vec[i];
    if (i + 1 < q_vec.size()) oss << ", ";
  }
  ROS_INFO_STREAM("q_vec = [" << oss.str() << "]");

  // Compute gravity torques
  if (dyn_->JntToGravity(q_, g_) < 0) {
    ROS_ERROR("KDL::JntToGravity failed");
    return;
  }

  // Optional: publish raw gravity torques (for plotting/verification)
  if (torque_pub_) {
    std_msgs::Float64MultiArray tau_msg;
    tau_msg.data.resize(g_.rows());
    for (unsigned i = 0; i < g_.rows(); ++i) tau_msg.data[i] = g_(i);
    torque_pub_.publish(tau_msg);
  }

  if (!enable_) return;  // allow running in “observe only” mode

  // Convert to motor current ticks (using your CurrentMap helper)
  std::vector<int16_t> ticks(q_vec.size(), 0);
  for (size_t i = 0; i < q_vec.size(); ++i) {
    CurrentMap m;
    m.K_T_EFF        = k_t_eff_[i];       // Nm/A (effective torque per amp)
    m.CURRENT_TICK_A = a_per_tick_[i];    // A / tick
    m.MAX_ABS_TICK   = max_abs_ticks_[i]; // clamp
    ticks[i] = torqueNmToGoalCurrentTick(g_(i), m);
  }

  std_msgs::Int16MultiArray cur_msg;
  cur_msg.data.assign(ticks.begin(), ticks.end());
  current_pub_.publish(cur_msg);
  // Pretty-print
  std::ostringstream oss_q, oss_tau, oss_named, oss_ticks;
  oss_q.setf(std::ios::fixed);   oss_q.precision(5);
  oss_tau.setf(std::ios::fixed); oss_tau.precision(5);
  oss_named.setf(std::ios::fixed); oss_named.precision(5);

  oss_q << "[";
  for (size_t i = 0; i < q_vec.size(); ++i)
    oss_q << q_vec[i] << (i + 1 < q_vec.size() ? ", " : "");
  oss_q << "]";

  oss_tau << "[";
  for (size_t i = 0; i < q_vec.size(); ++i)
    oss_tau << g_(i) << (i + 1 < q_vec.size() ? ", " : "");
  oss_tau << "]";

  for (size_t i = 0; i < chain_joint_names_.size(); ++i)
    oss_named << chain_joint_names_[i] << ": " << g_(i)
              << (i + 1 < chain_joint_names_.size() ? ", " : "");

  oss_ticks << "[";
  for (size_t i = 0; i < ticks.size(); ++i)
    oss_ticks << ticks[i] << (i + 1 < ticks.size() ? ", " : "");
  oss_ticks << "]";

  ROS_INFO_STREAM("\nq(rad)      = " << oss_q.str()
                << "\nτ_g (Nm)   = " << oss_tau.str()
                << "\nτ_g named  = { " << oss_named.str() << " }"
                << "\nticks      = " << oss_ticks.str());

}

bool GravityProbe::mapJointStateToChain(const sensor_msgs::JointState& js,
                                        const std::vector<std::string>& order,
                                        std::vector<double>* q_out) {
  if (q_out->size() != order.size()) q_out->resize(order.size(), 0.0);
  for (size_t i = 0; i < order.size(); ++i) {
    const auto& name = order[i];
    auto it = std::find(js.name.begin(), js.name.end(), name);
    if (it == js.name.end()) return false;
    const size_t idx = std::distance(js.name.begin(), it);
    if (idx >= js.position.size()) return false;
    (*q_out)[i] = js.position[idx];
  }
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gravity_cancellation");
  ros::NodeHandle nh, pnh("~");
  GravityProbe node(nh, pnh);
  ros::spin();
  return 0;
}
