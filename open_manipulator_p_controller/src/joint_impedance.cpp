#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>      // **** NEW **** FK
#include <kdl/chainjnttojacsolver.hpp>             // **** NEW **** Jacobian
#include <kdl/jacobian.hpp>

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
#include <string>

#include "open_manipulator_p_controller/current_map.h"
#include "open_manipulator_p_controller/joint_impedance.h"

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

ImpedanceProbe::ImpedanceProbe(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh) {
  // --- Params (same as before) ---
  pnh_.param<std::string>("urdf_param", urdf_param_, "/robot_description");
  pnh_.param<std::string>("base_link",  base_link_,  "open_manipulator_origin");
  pnh_.param<std::string>("tip_link",   tip_link_,   "open_manipulator_p_tool");
  pnh_.param<double>("gx", gx_, 0.0);
  pnh_.param<double>("gy", gy_, 0.0);
  pnh_.param<double>("gz", gz_, -9.81);

  std::string q_cli_s;
  pnh_.param<std::string>("q", q_cli_s, "");
  if (!q_cli_s.empty()) {
    cli_mode_ = parseCommaDoubles(q_cli_s, &q_cli_);
    if (!cli_mode_) ROS_ERROR("Failed to parse --q string.");
  }

  // this->q_hold_ << 0.0, -0.792, 0.397, 0.0, 0.393, 0.0;
  this->q_hold_ << 0.0, -1.618, 0.393, 0.002, 0.393, 0.0;


  pnh_.param("enable", enable_, true);
  pnh_.param("scale",  scale_,  1.0);
  pnh_.param<std::string>("current_topic", current_topic_, "goal_currents_ticks");
  pnh_.param<std::string>("torque_topic",  torque_topic_,  "impedance_torque");

  // **** NEW **** gains (override with params if you want)
  std::vector<double> Kxyz, Dxyz, Krot, Drot;
  if (pnh_.getParam("K_xyz", Kxyz) && Kxyz.size()==3) K_xyz_ = KDL::Vector(Kxyz[0],Kxyz[1],Kxyz[2]);
  if (pnh_.getParam("D_xyz", Dxyz) && Dxyz.size()==3) D_xyz_ = KDL::Vector(Dxyz[0],Dxyz[1],Dxyz[2]);
  if (pnh_.getParam("K_rot", Krot) && Krot.size()==3) K_rot_ = KDL::Vector(Krot[0],Krot[1],Krot[2]);
  if (pnh_.getParam("D_rot", Drot) && Drot.size()==3) D_rot_ = KDL::Vector(Drot[0],Drot[1],Drot[2]);

  current_pub_ = pnh_.advertise<std_msgs::Int16MultiArray>(current_topic_, 10);
  torque_pub_  = pnh_.advertise<std_msgs::Float64MultiArray>(torque_topic_, 10);

  // --- Load URDF and build KDL chain ---
  std::string urdf_xml;
  if (!nh_.getParam(urdf_param_, urdf_xml)) { ROS_ERROR_STREAM("URDF not found: " << urdf_param_); ros::shutdown(); return; }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree)) { ROS_ERROR("Failed to parse URDF"); ros::shutdown(); return; }

  if (!tree.getChain(base_link_, tip_link_, chain_)) {
    ROS_ERROR_STREAM("Failed to make chain " << base_link_ << " -> " << tip_link_); ros::shutdown(); return;
  }

  // Joint name order
  chain_joint_names_.clear();
  for (unsigned i=0;i<chain_.getNrOfSegments();++i) {
    const auto& j = chain_.getSegment(i).getJoint();
    if (j.getType()!=KDL::Joint::None) chain_joint_names_.push_back(j.getName());
  }

  // KDL solvers
  dyn_.reset(new KDL::ChainDynParam(chain_, KDL::Vector(gx_,gy_,gz_)));
  fk_.reset(new KDL::ChainFkSolverPos_recursive(chain_));   // **** NEW ****
  jac_solve_.reset(new KDL::ChainJntToJacSolver(chain_));   // **** NEW ****

  q_.resize(chain_.getNrOfJoints());
  dq_.resize(chain_.getNrOfJoints());
  g_.resize(chain_.getNrOfJoints());
  J_.resize(chain_.getNrOfJoints());

  // Motor constants (same mapping as your code)
  const size_t N = chain_.getNrOfJoints();
  k_t_eff_.resize(N); a_per_tick_.resize(N); max_abs_ticks_.resize(N);

  enum class Model { PH54_200, PH54_100, PH42_020 };
  std::vector<Model> model_of_joint = {
    Model::PH54_200, Model::PH54_200, Model::PH54_100,
    Model::PH54_100, Model::PH42_020, Model::PH42_020
  };
  
  max_abs_ticks_ = {22740,22740,15900,15900,4500,4500};

  for (size_t i=0;i<N;++i) {
    switch (model_of_joint[i]) {
      case Model::PH54_200: a_per_tick_[i]=0.001; k_t_eff_[i]=4.81; break;
      case Model::PH54_100: a_per_tick_[i]=0.001; k_t_eff_[i]=4.60; break;
      case Model::PH42_020: a_per_tick_[i]=0.001; k_t_eff_[i]=3.40; break;
    }
  }

  ROS_INFO_STREAM("ImpedanceProbe DOF="<<N<<" base='"<<base_link_<<"' tip='"<<tip_link_<<"'");

  // CLI one-shot (position only; velocity=0)
  if (cli_mode_) {
    if (q_cli_.size()!=N) { ROS_ERROR_STREAM("CLI q size "<<q_cli_.size()<<" != "<<N); ros::shutdown(); return; }
    std::vector<double> dq0(N,0.0);
    computeAndPublish(q_cli_, dq0);
  }

  // Live mode
  sub_js_ = nh_.subscribe("/joint_states", 1, &ImpedanceProbe::jointCb, this);
  sub_xd_ = nh_.subscribe("/impedance/desired_pose", 1, &ImpedanceProbe::xdCb, this); // **** NEW ****
  sub_vd_ = nh_.subscribe("/impedance/desired_twist",1, &ImpedanceProbe::vdCb, this);  // **** NEW ****

  timer_  = nh_.createTimer(ros::Duration(0.002), &ImpedanceProbe::timerCb, this); // 500 Hz typical inner loop
}

void ImpedanceProbe::xdCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  xd_msg_ = *msg; have_xd_ = true;
}

void ImpedanceProbe::vdCb(const geometry_msgs::Twist::ConstPtr& msg) {
  vd_msg_ = *msg; have_vd_ = true;
}

void ImpedanceProbe::jointCb(const sensor_msgs::JointState::ConstPtr& msg) {
  last_js_ = *msg;
  have_js_ = true;
}

void ImpedanceProbe::timerCb(const ros::TimerEvent&) {
  if (!have_js_) return;

  std::vector<double> q(chain_joint_names_.size(), 0.0), dq(chain_joint_names_.size(), 0.0);
  if (!mapJointStateToChain(last_js_, chain_joint_names_, &q, &dq)) return;

  computeAndPublish(q, dq);
}

// Small-angle orientation error in world frame.
// e_rot = 0.5 * vee( R_d^T * R - (R_d^T * R)^T )
static KDL::Vector rotVecError(const KDL::Rotation& R_now,
                               const KDL::Rotation& R_des)
{
  // A = R_d^T * R
  KDL::Rotation A = R_des.Inverse() * R_now;

  // vee(A - A^T) / 2
  double ex = 0.5 * (A(2,1) - A(1,2));
  double ey = 0.5 * (A(0,2) - A(2,0));
  double ez = 0.5 * (A(1,0) - A(0,1));
  return KDL::Vector(ex, ey, ez);
}

void ImpedanceProbe::computeAndPublish(const std::vector<double>& q_vec,
                                       const std::vector<double>& dq_vec,
                                       double dt)
{
  const int N = n_joints_;
  // Map std::vector -> Eigen
  Eigen::VectorXd q(N), dq(N);
  for (int i=0; i<N; ++i) { q(i) = q_vec[i]; dq(i) = dq_vec[i]; }

  // One-time init / teach
  if (!have_q_hold_) {
    q_hold_    = q;
    dq_filt_   = Eigen::VectorXd::Zero(N);
    tau_prev_  = Eigen::VectorXd::Zero(N);
    // Reasonable starter gains for a small arm
    Kq_ = (Eigen::VectorXd(N) << 8, 8, 6, 3, 2, 1).finished();                // Nm/rad
    Dq_ = 2.0 * Kq_.cwiseSqrt();                                              // ~critical
    // Optional torque limits
    tau_limits_ = (Eigen::VectorXd(N) << 6, 6, 4, 3, 1.5, 1.0).finished();    // Nm
    have_q_hold_ = true;
  }

  // Low-pass filter dq (first-order)
  const double wc = 2.0 * M_PI * vel_cutoff_hz_;
  const double alpha = (dt*wc) / (1.0 + dt*wc); // 0..1
  dq_filt_ = (1.0 - alpha) * dq_filt_ + alpha * dq;

  // Errors (derivative-on-measurement)
  Eigen::VectorXd e_q  = q_hold_ - q;        // [rad]
  Eigen::VectorXd ed_q = -dq_filt_;          // [rad/s]

  // Gravity compensation (KDL)
  Eigen::VectorXd gq = Eigen::VectorXd::Zero(N);
  if (dyn_) {
    KDL::JntArray q_kdl(N), g_kdl(N);
    for (int i=0; i<N; ++i) q_kdl(i) = q(i);
    dyn_->JntToGravity(q_kdl, g_kdl);
    for (int i=0; i<N; ++i) gq(i) = g_kdl(i);     // Nm
  }

  // Joint-space impedance torque
  Eigen::VectorXd tau = Kq_.cwiseProduct(e_q) + Dq_.cwiseProduct(ed_q) + gq;

  // Clamp per-joint torque (optional)
  if (tau_limits_.size() == N) {
    for (int i=0; i<N; ++i) {
      tau(i) = std::max(-tau_limits_(i), std::min(tau(i), tau_limits_(i)));
    }
  }

  // Rate limit (optional)
  if (max_tau_rate_ > 0.0) {
    const double max_step = max_tau_rate_ * dt;  // Nm per cycle
    for (int i=0; i<N; ++i) {
      const double d = tau(i) - tau_prev_(i);
      const double d_clamped = std::max(-max_step, std::min(d, max_step));
      tau(i) = tau_prev_(i) + d_clamped;
    }
  }
  tau_prev_ = tau;

  // --- Convert Nm -> current ticks (fill with YOUR real constants) ---
  std::vector<int16_t> ticks(N, 0);
  for (int i=0; i<N; ++i) {
    const double kt   = (i < (int)k_t_eff_.size())   ? k_t_eff_[i]   : 1.0;      // Nm/A
    const double a_pt = (i < (int)a_per_tick_.size())? a_per_tick_[i]: 1.0/2048; // A/tick (example)
    double current_A  = tau(i) / kt;                   // A
    double raw_ticks  = current_A / a_pt;              // ticks
    long t = std::lround(raw_ticks);

    // Optional hardware safety clamp
    if (i < (int)max_abs_ticks_.size() && max_abs_ticks_[i] > 0) {
      t = std::max(-max_abs_ticks_[i], std::min(t, max_abs_ticks_[i]));
    }
    ticks[i] = static_cast<int16_t>(t);
  }

  // --- Publish ---
  std_msgs::Int16MultiArray m;
  m.data.resize(N);
  for (int i=0; i<N; ++i) m.data[i] = ticks[i];
  current_pub_.publish(m);

  // (Optional) Debug once in a while
  if ((ros::Time::now().toNSec()/1000000) % 200 == 0) {
    ROS_INFO_STREAM_THROTTLE(0.5,
      "JS-Imp: |e_q|=" << e_q.norm()
      << " |ed_q|=" << ed_q.norm()
      << " |tau|="  << tau.norm());
  }
}


bool ImpedanceProbe::mapJointStateToChain(const sensor_msgs::JointState& js,
                                          const std::vector<std::string>& order,
                                          std::vector<double>* q_out,
                                          std::vector<double>* dq_out) {
  if (q_out->size()!=order.size()) q_out->resize(order.size(),0.0);
  if (dq_out->size()!=order.size()) dq_out->resize(order.size(),0.0);
  for (size_t i=0;i<order.size(); ++i) {
    const auto& name = order[i];
    auto it = std::find(js.name.begin(), js.name.end(), name);
    if (it == js.name.end()) return false;
    const size_t idx = std::distance(js.name.begin(), it);
    if (idx >= js.position.size()) return false;
    (*q_out)[i]  = js.position[idx];
    if (idx < js.velocity.size()) (*dq_out)[i] = js.velocity[idx]; else (*dq_out)[i]=0.0;
  }
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_impedance_controller");
  ros::NodeHandle nh, pnh("~");
  ImpedanceProbe node(nh, pnh);
  ros::spin();
  return 0;
}
