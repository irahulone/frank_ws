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
#include <cmath>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
#include <string>

#include "open_manipulator_p_controller/current_map.h"
#include "open_manipulator_p_controller/cartesian_impedance.h"

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


KDL::Vector ImpedanceProbe::rotVecError(const KDL::Rotation& R_now,
                               const KDL::Rotation& R_des)
{
    // Rotation error taking you from current to desired
    KDL::Rotation R_err = R_des * R_now.Inverse();  // or R_des * R_now^T
    KDL::Vector axis = R_err.GetRot();  // Gets rotation axis
    double angle = R_err.GetRotAngle(axis);  // Gets angle and updates axis
    // Proper angle wrapping to [-π, π]
    angle = std::atan2(std::sin(angle), std::cos(angle));

    // For small angles, use small-angle approximation
    const double eps = 1e-6;
    if (std::abs(angle) < eps) {
        // Small angle approximation using trace
        const double r01 = R_err(0,1), r02 = R_err(0,2);
        const double r10 = R_err(1,0), r12 = R_err(1,2);
        const double r20 = R_err(2,0), r21 = R_err(2,1);
        return KDL::Vector( 0.5*(r21 - r12),
                            0.5*(r02 - r20),
                            0.5*(r10 - r01) );
    }

    return axis * angle;
}



void ImpedanceProbe::computeAndPublish(const std::vector<double>& q_vec,
                                       const std::vector<double>& dq_vec) {
  const size_t N = q_vec.size();
  for (size_t i=0;i<N;++i){ q_(i)=q_vec[i]; dq_(i)=dq_vec[i]; }

  // --- FK & Jacobian ---
  if (fk_->JntToCart(q_, x_now_) < 0) { ROS_ERROR_THROTTLE(1.0,"FK failed"); return; }
  if (jac_solve_->JntToJac(q_, J_) < 0) { ROS_ERROR_THROTTLE(1.0,"Jacobian failed"); return; }
  std::ostringstream ss;
  ss << "Jacobian J_:\n";
  for (unsigned int r = 0; r < J_.rows(); ++r) {
    for (unsigned int c = 0; c < J_.columns(); ++c) {
      ss << J_(r, c) << " ";
    }
    ss << "\n";
  }

  ROS_INFO_STREAM(ss.str());
  // Desired pose: hold initial pose if none provided
  KDL::Frame x_des;
  if (have_xd_) {
    const auto& p = xd_msg_.pose.position;
    const auto& o = xd_msg_.pose.orientation;
    x_des = KDL::Frame(
      KDL::Rotation::Quaternion(o.x,o.y,o.z,o.w),
      KDL::Vector(p.x,p.y,p.z));
  } else {
    if (!have_x_hold_) { x_des = x_now_; xd_msg_.pose.orientation.w=1; have_x_hold_=true; }
    else x_des = x_now_;
  }

  // Desired tool twist (feedforward); defaults to 0
  KDL::Twist v_des(
    KDL::Vector( have_vd_?vd_msg_.linear.x:0.0,
                 have_vd_?vd_msg_.linear.y:0.0,
                 have_vd_?vd_msg_.linear.z:0.0 ),
    KDL::Vector( have_vd_?vd_msg_.angular.x:0.0,
                 have_vd_?vd_msg_.angular.y:0.0,
                 have_vd_?vd_msg_.angular.z:0.0 ));

  // Current tool twist via v = J * dq
  KDL::Twist v_now;
  {
    // J is 6xN; multiply by dq
    KDL::Vector lin(0,0,0), ang(0,0,0);
    for (unsigned j=0;j<N;++j) {
      ang[0] += J_(0,j)*dq_(j);
      ang[1] += J_(1,j)*dq_(j);
      ang[2] += J_(2,j)*dq_(j);
      lin[0] += J_(3,j)*dq_(j);
      lin[1] += J_(4,j)*dq_(j);
      lin[2] += J_(5,j)*dq_(j);
    }
    v_now = KDL::Twist(lin, ang);
  }

  // --- Task-space spring-damper ---
  // pos error in tool/world frame
  KDL::Vector e_p = x_des.p - x_now_.p;
  // small-angle orientation error (in world frame)
  KDL::Vector e_o = rotVecError(x_now_.M, x_des.M);

  // vel error
  KDL::Vector ev_lin = KDL::Vector(v_des.vel.x(), v_des.vel.y(), v_des.vel.z()) - v_now.vel;
  KDL::Vector ev_rot = KDL::Vector(v_des.rot.x(), v_des.rot.y(), v_des.rot.z()) - v_now.rot;

  // Wrench = [F; M] = K*(e) + D*(e_dot)
  KDL::Wrench F_task(
    KDL::Vector( K_xyz_.x()*e_p.x() + D_xyz_.x()*ev_lin.x(),
                 K_xyz_.y()*e_p.y() + D_xyz_.y()*ev_lin.y(),
                 K_xyz_.z()*e_p.z() + D_xyz_.z()*ev_lin.z() ),
    KDL::Vector( K_rot_.x()*e_o.x() + D_rot_.x()*ev_rot.x(),
                 K_rot_.y()*e_o.y() + D_rot_.y()*ev_rot.y(),
                 K_rot_.z()*e_o.z() + D_rot_.z()*ev_rot.z() )
  );

  // --- Gravity torques ---
  if (dyn_->JntToGravity(q_, g_) < 0) { ROS_ERROR_THROTTLE(1.0,"JntToGravity failed"); return; }

  // --- Map wrench to joint torques: tau_task = J^T * wrench ---
  Eigen::VectorXd tau_task(N); tau_task.setZero();
  // KDL Jacobian stores rows as [wx wy wz vx vy vz]
  // Compose wrench vector W = [wx wy wz vx vy vz]^T matching that order:
  Eigen::Matrix<double,6,1> W;
  W << F_task.torque.x(), F_task.torque.y(), F_task.torque.z(),
       F_task.force.x(),  F_task.force.y(),  F_task.force.z();
  
  Eigen::MatrixXd Je(6,N);
  for (unsigned r=0;r<6;++r) for (unsigned c=0;c<N;++c) Je(r,c)=J_(r,c);
  tau_task = Je.transpose()*W;

  // --- Total torque ---
  Eigen::VectorXd tau(N);
  for (unsigned i=0;i<N;++i) tau(i) = tau_task(i) + g_(i); // add gravity comp

  // Optional scaling/saturation
  tau *= scale_;

  // Publish debug torques
  if (torque_pub_) {
    std_msgs::Float64MultiArray tmsg;
    tmsg.data.resize(N);
    for (unsigned i=0;i<N;++i) tmsg.data[i]=tau(i);
    torque_pub_.publish(tmsg);
  }

  if (!enable_) return;

  // --- Convert to current ticks (exactly your path) ---
  std::vector<int16_t> ticks(N,0);
  for (size_t i=0;i<N;++i){
    CurrentMap m; 
    m.K_T_EFF=k_t_eff_[i]; 
    m.CURRENT_TICK_A=a_per_tick_[i]; 
    m.MAX_ABS_TICK=max_abs_ticks_[i];
    ticks[i] = torqueNmToGoalCurrentTick(tau(i), m);
  }
  std_msgs::Int16MultiArray cur_msg;
  cur_msg.data.assign(ticks.begin(), ticks.end());
  current_pub_.publish(cur_msg);
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
  ros::init(argc, argv, "cartesian_impedance_controller");
  ros::NodeHandle nh, pnh("~");
  ImpedanceProbe node(nh, pnh);
  ros::spin();
  return 0;
}
