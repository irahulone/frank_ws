#include <ros/ros.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "torque_control_node");
  ros::NodeHandle nh;

  std::string port_name = "/dev/ttyUSB0";
  int baud_rate = 1000000;

  // IDs of your motors
  std::vector<uint8_t> motor_ids = {1, 2, 3, 4, 5, 6};
  std::vector<int16_t> goal_currents = {300, 300, 300, 300, 10, 30};
  DynamixelWorkbench dxl_wb;
  const char* log;

  // Initialize DYNAMIXEL
  if (!dxl_wb.init(port_name.c_str(), baud_rate)) {
    ROS_ERROR("Failed to initialize port");
    return 1;
  }

  for (size_t i = 0; i < motor_ids.size(); ++i)
  {
    uint8_t id = motor_ids[i];
    int16_t goal_current = goal_currents[i];
    uint16_t model_number;
    const char* model_name;

    // Ping motor
    if (!dxl_wb.ping(id, &model_number, &model_name)) {
      ROS_WARN("Failed to ping motor ID %d", id);
      continue;
    }
    ROS_INFO("Connected to motor ID %d: Model %s (%d)", id, model_name, model_number);

    // Set to Torque Control Mode (0)
    if (!dxl_wb.itemWrite(id, "Operating_Mode", 0, &log)) {
      ROS_ERROR("ID %d - Failed to set Torque Control Mode: %s", id, log);
      continue;
    }

    // Enable Torque
    if (!dxl_wb.itemWrite(id, "Torque_Enable", 1, &log)) {
      ROS_ERROR("ID %d - Failed to enable torque: %s", id, log);
      continue;
    }

    // Apply Goal Current (e.g., 300 = ~0.81A)
//    if (!dxl_wb.itemWrite(id, "Goal_Current", goal_current, &log)) {
  //    ROS_ERROR("ID %d - Failed to write Goal Current: %s", id, log);
    //  continue;
   // }

    //ROS_INFO("Motor ID %d set to Torque Mode with Goal_Current = %d", id, goal_current);
  }

  ros::spin(); // Keep node alive (optional)

  return 0;
}

