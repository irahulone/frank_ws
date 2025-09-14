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

#ifndef OPEN_MANIPULATOR_P_CONTROLLER_H_
#define OPEN_MANIPULATOR_P_CONTROLLER_H_

#include <boost/thread.hpp>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "open_manipulator_p_libs/open_manipulator_p.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetDrawingTrajectory.h"
#include "open_manipulator_msgs/SetActuatorState.h"
#include "open_manipulator_msgs/GetJointPosition.h"
#include "open_manipulator_msgs/GetKinematicsPose.h"
#include "open_manipulator_msgs/OpenManipulatorState.h"
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

namespace open_manipulator_p_controller
{
class OmTorqueController
{
 public:
  OmTorqueController(std::string usb_port, std::string baud_rate);
  ~OmTorqueController();

  // update
  void publishCallback(const ros::TimerEvent&);
  void startTimerThread();
  static void *timerThread(void *param);
  void process(double time);
  double getControlPeriod(void){return control_period_;}

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/
  bool using_platform_;
  double control_period_;
  bool with_gripper_;
  std::string actuator_mode_;


  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  // Thread parameter
  pthread_t timer_thread_;
  pthread_attr_t attr_;
  bool timer_thread_state_;

  // Related robotis_manipulator
  OpenManipulator open_manipulator_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initPublisher();
  void initSubscriber();
  void initServer();

  /*****************************************************************************
  ** ROS Publishers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Publisher open_manipulator_states_pub_;
  std::vector<ros::Publisher> open_manipulator_kinematics_pose_pub_;
  ros::Publisher open_manipulator_joint_states_pub_;
  std::vector<ros::Publisher> gazebo_goal_joint_position_pub_;
  ros::Publisher torque_pub_;
  ros::Publisher goal_current_pub_;


  void publishOpenManipulatorStates();
  void publishKinematicsPose();
  void publishJointStates();
  void publishGazeboCommand();

  /*****************************************************************************
  ** ROS Subscribers and Callback Functions
  *****************************************************************************/
  ros::Subscriber open_manipulator_option_sub_;

  void openManipulatorOptionCallback(const std_msgs::String::ConstPtr &msg);



  std::vector<int16_t> goal_currents_;


 protected:
  const char* log;
};
}
#endif //OPEN_MANIPULATOR_P_CONTROLLER_H_

