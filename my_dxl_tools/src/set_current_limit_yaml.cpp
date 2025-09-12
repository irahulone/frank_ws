#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_current_limit_yaml");

  std::string port_name = "/dev/ttyUSB0";
  int baud_rate = 1000000;
  std::string yaml_path = "/home/open-arm/test_frank_ws/src/my_dxl_tools/config/current_limit.yaml";

  YAML::Node config = YAML::LoadFile(yaml_path);
  std::cout << "==== YAML FILE CONTENT ====" << std::endl;
  std::ifstream file(yaml_path);
  std::string line;
  while (std::getline(file, line)) {
    std::cout << line << std::endl;
  }
  std::cout << "===========================" << std::endl;

  if (!config["current_limit"]) {
    ROS_ERROR("Missing 'current_limit' in YAML file.");
    return 1;
  }

  DynamixelWorkbench dxl_wb;
  if (!dxl_wb.init(port_name.c_str(), uint32_t(baud_rate))) {
    ROS_ERROR("Failed to open port");
    return 1;
  }

  for (auto joint : config["current_limit"])
  {
    int id_int = joint.first.as<int>();
    if (id_int < 1 || id_int > 253) {
      ROS_WARN("Skipping invalid ID: %d", id_int);
      continue;
    }
    uint8_t id = static_cast<uint8_t>(id_int);
    uint16_t current_limit = joint.second.as<uint16_t>();

    uint16_t model_number;
    const char* model_name;

    if (!dxl_wb.ping(id, &model_number, &model_name)) {
      ROS_ERROR("Failed to ping ID %d", id);
      continue;
    }

    ROS_INFO("ID %d: Model Number: %d, Model Name: %s", id, model_number, model_name);

    const char* log;
    if (!dxl_wb.itemWrite(id, "Current_Limit", current_limit, &log)) {
      ROS_ERROR("Failed to write Current_Limit to ID %d: %s", id, log);
    } else {
      ROS_INFO("Successfully set Current_Limit for ID %d: %d mA", id, current_limit);
    }
  }

  return 0;
}

