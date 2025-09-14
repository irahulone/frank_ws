/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "open_manipulator_p_controller/om_torque_controller.h"

using namespace open_manipulator_p_controller;

OmTorqueController::OmTorqueController(std::string usb_port, std::string baud_rate)
: node_handle_(""),
  priv_node_handle_("~"),
  timer_thread_state_(false),
  with_gripper_(false)
{

  this->goal_currents_ = {0, 0, 0, 0, 0, 150};

  /************************************************************
  ** Initialize ROS parameters
  ************************************************************/
  control_period_       = priv_node_handle_.param<double>("control_period", 0.010f);
  using_platform_       = priv_node_handle_.param<bool>("using_platform", true);
  with_gripper_         = priv_node_handle_.param<bool>("with_gripper", false);
  actuator_mode_        = priv_node_handle_.param<std::string>("actuator_mode", "position_mode");
  std::string currents_in;
  priv_node_handle_.param<std::string>("goal_currents_ticks_in", currents_in, "goal_currents_ticks_in");

  /************************************************************
  ** Initialize variables
  ************************************************************/
  open_manipulator_.initOpenManipulator(using_platform_, usb_port, baud_rate, control_period_, with_gripper_, actuator_mode_);

  if (using_platform_ == true) log::info("Succeeded to init " + priv_node_handle_.getNamespace());
  else if (using_platform_ == false) log::info("Ready to simulate " + priv_node_handle_.getNamespace() + " on Gazebo");

  /************************************************************
  ** Initialize ROS publishers, subscribers and servers
  ************************************************************/
  initPublisher();
  initSubscriber();
}

OmTorqueController::~OmTorqueController()
{
  timer_thread_state_ = false;
  pthread_join(timer_thread_, NULL); // Wait for the thread associated with thread_p to complete
  log::info("Shutdown the OpenManipulator");
  open_manipulator_.disableAllActuator();
  ros::shutdown();
}

void OmTorqueController::startTimerThread()
{
  ////////////////////////////////////////////////////////////////////
  /// Use this when you want to increase the priority of threads.
  ////////////////////////////////////////////////////////////////////
  //  pthread_attr_t attr_;
  //  int error;
  //  struct sched_param param;
  //  pthread_attr_init(&attr_);

  //  error = pthread_attr_setschedpolicy(&attr_, SCHED_RR);
  //  if (error != 0)   log::error("pthread_attr_setschedpolicy error = ", (double)error);
  //  error = pthread_attr_setinheritsched(&attr_, PTHREAD_EXPLICIT_SCHED);
  //  if (error != 0)   log::error("pthread_attr_setinheritsched error = ", (double)error);

  //  memset(&param, 0, sizeof(param));
  //  param.sched_priority = 31;    // RT
  //  error = pthread_attr_setschedparam(&attr_, &param);
  //  if (error != 0)   log::error("pthread_attr_setschedparam error = ", (double)error);

  //  if ((error = pthread_create(&this->timer_thread_, &attr_, this->timerThread, this)) != 0)
  //  {
  //    log::error("Creating timer thread failed!!", (double)error);
  //    exit(-1);
  //  }
  // timer_thread_state_ = true;
  ////////////////////////////////////////////////////////////////////

  int error;
  if ((error = pthread_create(&this->timer_thread_, NULL, this->timerThread, this)) != 0)
  {
    log::error("Creating timer thread failed!!", (double)error);
    exit(-1);
  }
  timer_thread_state_ = true;
}

void *OmTorqueController::timerThread(void *param)
{
  OmTorqueController *controller = (OmTorqueController *) param;
  static struct timespec next_time;
  static struct timespec curr_time;

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while(controller->timer_thread_state_)
  {
    next_time.tv_sec += (next_time.tv_nsec + ((int)(controller->getControlPeriod() * 1000)) * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + ((int)(controller->getControlPeriod() * 1000)) * 1000000) % 1000000000;

    double time = next_time.tv_sec + (next_time.tv_nsec*0.000000001);
    controller->process(time);

    clock_gettime(CLOCK_MONOTONIC, &curr_time);
    /////
    double delta_nsec = controller->getControlPeriod() - ((next_time.tv_sec - curr_time.tv_sec) + ((double)(next_time.tv_nsec - curr_time.tv_nsec)*0.000000001));
    // log::info("control time : ", controller->getControlPeriod() - delta_nsec);
    if (delta_nsec > controller->getControlPeriod())
    {
      log::warn("Over the control time : ", delta_nsec);
      next_time = curr_time;
    }
    else
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    /////
  }
  return 0;
}

/********************************************************************************
** Init Functions
********************************************************************************/
void OmTorqueController::initPublisher()
{
  // ros message publisher
  auto om_tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  for (auto const& name:om_tools_name)
  {
    ros::Publisher pb;
    pb = node_handle_.advertise<open_manipulator_msgs::KinematicsPose>(name + "/kinematics_pose", 10);
    open_manipulator_kinematics_pose_pub_.push_back(pb);
  }
  open_manipulator_states_pub_ = node_handle_.advertise<open_manipulator_msgs::OpenManipulatorState>("states", 10);

  if (using_platform_ == true)
  {
    open_manipulator_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);
  }
  else
  {
    auto gazebo_joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
    gazebo_joints_name.reserve(gazebo_joints_name.size() + om_tools_name.size());
    gazebo_joints_name.insert(gazebo_joints_name.end(), om_tools_name.begin(), om_tools_name.end());

    for (auto const& name:gazebo_joints_name)
    {
      ros::Publisher pb;
      pb = node_handle_.advertise<std_msgs::Float64>(name + "_position/command", 10);
      gazebo_goal_joint_position_pub_.push_back(pb);
    }
  }
}


void OmTorqueController::initSubscriber()
{
  // ros message subscriber
  open_manipulator_option_sub_ = node_handle_.subscribe("option", 10, &OmTorqueController::openManipulatorOptionCallback, this);
}

/*****************************************************************************
** Callback Functions for ROS Subscribers
*****************************************************************************/
void OmTorqueController::openManipulatorOptionCallback(const std_msgs::String::ConstPtr &msg)
{
  if (msg->data == "print_open_manipulator_p_setting")
    open_manipulator_.printManipulatorSetting();
}

/********************************************************************************
** Callback function for process timer
********************************************************************************/
void OmTorqueController::process(double time)
{
  open_manipulator_.processOpenManipulatorTorqueOnly(this->goal_currents_);
}

/********************************************************************************
** Callback function for publish timer
********************************************************************************/
void OmTorqueController::publishCallback(const ros::TimerEvent&)
{
  if (using_platform_ == true) publishJointStates();
  else publishGazeboCommand();

  publishOpenManipulatorStates();
  publishKinematicsPose();
  publishTaskWrench();
}

void OmTorqueController::publishOpenManipulatorStates()
{
  open_manipulator_msgs::OpenManipulatorState msg;
  if (open_manipulator_.getMovingState())
    msg.open_manipulator_moving_state = msg.IS_MOVING;
  else
    msg.open_manipulator_moving_state = msg.STOPPED;

  if (open_manipulator_.getActuatorEnabledState(JOINT_DYNAMIXEL))
    msg.open_manipulator_actuator_state = msg.ACTUATOR_ENABLED;
  else
    msg.open_manipulator_actuator_state = msg.ACTUATOR_DISABLED;

  open_manipulator_states_pub_.publish(msg);
}

void OmTorqueController::publishTaskWrench()
{
  auto joint_value = open_manipulator_.getAllActiveJointValue();
  auto joint_names = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
  std::string tool_name = open_manipulator_.getManipulator()->getAllToolComponentName().front();

  // 1. Extract joint efforts into an Eigen vector
  Eigen::VectorXd tau(joint_value.size());
  for (size_t i = 0; i < joint_value.size(); ++i)
    tau(i) = joint_value.at(i).effort;

  // 2. Get Jacobian (this assumes your class has the method like this)
  Eigen::MatrixXd J = open_manipulator_.jacobian("joint1");
  
  // 3. Compute wrench = J^T * tau
//  Eigen::VectorXd wrench = J.transpose() * tau;
  Eigen::VectorXd wrench = (J.transpose()).completeOrthogonalDecomposition().pseudoInverse() * tau;

  // 4. Fill and publish geometry_msgs::WrenchStamped
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = ros::Time::now();
  wrench_msg.header.frame_id = tool_name;

  // Assume wrench is 6x1: [Fx, Fy, Fz, Tx, Ty, Tz]
  wrench_msg.wrench.force.x = wrench(0);
  wrench_msg.wrench.force.y = wrench(1);
  wrench_msg.wrench.force.z = wrench(2);
  wrench_msg.wrench.torque.x = wrench(3);
  wrench_msg.wrench.torque.y = wrench(4);
  wrench_msg.wrench.torque.z = wrench(5);

  //ROS_INFO_STREAM("ee force:"<< wrench_msg);

  task_wrench_pub_.publish(wrench_msg);
}

void OmTorqueController::publishKinematicsPose()
{
  open_manipulator_msgs::KinematicsPose msg;
  auto om_tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  uint8_t index = 0;
  for (auto const& tools:om_tools_name)
  {
    KinematicPose pose = open_manipulator_.getKinematicPose(tools);
    msg.pose.position.x = pose.position[0];
    msg.pose.position.y = pose.position[1];
    msg.pose.position.z = pose.position[2];
    Eigen::Quaterniond orientation = math::convertRotationMatrixToQuaternion(pose.orientation);
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();

    open_manipulator_kinematics_pose_pub_.at(index).publish(msg);
    index++;
  }
}

void OmTorqueController::publishJointStates()
{
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();

  auto joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
  auto tool_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  auto joint_value = open_manipulator_.getAllActiveJointValue();
  auto tool_value = open_manipulator_.getAllToolValue();

  for (uint8_t i = 0; i < joints_name.size(); i ++)
  {
    msg.name.push_back(joints_name.at(i));

    msg.position.push_back(joint_value.at(i).position);
    msg.velocity.push_back(joint_value.at(i).velocity);
    msg.effort.push_back(joint_value.at(i).effort);
  }

  for (uint8_t i = 0; i < tool_name.size(); i ++)
  {
    msg.name.push_back(tool_name.at(i));

    msg.position.push_back(tool_value.at(i).position);
    msg.velocity.push_back(0.0f);
    msg.effort.push_back(0.0f);
  }
  open_manipulator_joint_states_pub_.publish(msg);
}

void OmTorqueController::publishGazeboCommand()
{
  JointWaypoint joint_value = open_manipulator_.getAllActiveJointValue();
  JointWaypoint tool_value = open_manipulator_.getAllToolValue();
  
  // angle (rad) -> distance (m)
  // tool_value.at(0).position = (1.135 - tool_value.at(0).position) / 1.135 * 0.109;

  for (uint8_t i = 0; i < joint_value.size(); i ++)
  {
    std_msgs::Float64 msg;
    msg.data = joint_value.at(i).position;

    gazebo_goal_joint_position_pub_.at(i).publish(msg);
  }

  for (uint8_t i = 0; i < tool_value.size(); i ++)
  {
    std_msgs::Float64 msg;
    msg.data = tool_value.at(i).position;

    gazebo_goal_joint_position_pub_.at(joint_value.size() + i).publish(msg);
  }
}

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char **argv)
{
  // init
  ros::init(argc, argv, "open_manipulator_p_controller");
  ros::NodeHandle node_handle("");

  std::string usb_port = "/dev/ttyUSB0";
  std::string baud_rate = "1000000";

  if (argc = 3)
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

  OmTorqueController om_controller(usb_port, baud_rate);

  // update
  om_controller.startTimerThread();
  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(om_controller.getControlPeriod()), &OmTorqueController::publishCallback, &om_controller);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}





