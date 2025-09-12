#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_pid_gains_yaml");

  std::string port_name = "/dev/ttyUSB0";
  int baud_rate = 1000000;
  std::string yaml_path1 = "/home/open-arm/frank_combined_ws/src/my_dxl_tools/config/pid_gains.yaml";

  YAML::Node config = YAML::LoadFile(yaml_path1);
  std::cout << "==== YAML FILE CONTENT ====" << std::endl;
  std::ifstream file(yaml_path1);
  std::string line;
  while (std::getline(file, line)) {
    std::cout << line << std::endl;
  }
  std::cout << "===========================" << std::endl;

  if (!config["pid_gains"]) {
    ROS_ERROR("Missing 'pid_gains' in YAML file.");
    return 1;
  }

  DynamixelWorkbench dxl_wb;
  if (!dxl_wb.init(port_name.c_str(), uint32_t(baud_rate))) {
    ROS_ERROR("Failed to open port");
    return 1;
  }
  

  for (auto joint : config["pid_gains"])
  {
    int id_int = joint.first.as<int>();
    if (id_int < 1 || id_int > 253) {
      ROS_WARN("Skipping invalid ID: %d", id_int);
      continue;
    }
    uint8_t id = static_cast<uint8_t>(id_int);
    ROS_INFO("Loading PID config for ID: %d", id);

    uint16_t p_gain = joint.second["p"].as<uint16_t>();
    uint16_t i_gain = joint.second["i"].as<uint16_t>();
    uint16_t d_gain = joint.second["d"].as<uint16_t>();
    uint16_t vp_gain = joint.second["vp"].as<uint16_t>();
    uint16_t vi_gain = joint.second["vi"].as<uint16_t>();
    uint16_t f1_gain = joint.second["f1"].as<uint16_t>();
    uint16_t f2_gain = joint.second["f2"].as<uint16_t>();

    //  ROS_INFO("Loading PID config for ID: %d", joint.first.as<uint8_t>());

    uint16_t model_number;
    const char* model_name;

    if (!dxl_wb.ping(id, &model_number, &model_name)) {
      ROS_ERROR("Failed to ping ID %d", id);
      continue;
    }

    ROS_INFO("ID %d: Model Number: %d, Model Name: %s", id, model_number, model_name);

    // Optional: Uncomment to force into position control mode
    // if (!dxl_wb.itemWrite(id, "Operating_Mode", 3)) {
    //   ROS_WARN("Failed to set Operating_Mode to Position Control for ID %d", id);
    // }

    const char* log;
    bool success = true;
    success &= dxl_wb.itemWrite(id, "Position_P_Gain", p_gain, &log);
    if (!success) ROS_ERROR("ID %d - Failed to write P Gain: %s", id, log);
    success &= dxl_wb.itemWrite(id, "Position_I_Gain", i_gain, &log);
    if (!success) ROS_ERROR("ID %d - Failed to write I Gain: %s", id, log);
    success &= dxl_wb.itemWrite(id, "Position_D_Gain", d_gain, &log);
    if (!success) ROS_ERROR("ID %d - Failed to write D Gain: %s", id, log);
    success &= dxl_wb.itemWrite(id, "Velocity_P_Gain", vp_gain, &log);
    if (!success) ROS_ERROR("ID %d - Failed to write velocity P Gain: %s", id, log);
    success &= dxl_wb.itemWrite(id, "Velocity_I_Gain",vi_gain , &log);
    if (!success) ROS_ERROR("ID %d - Failed to write velocity I Gain: %s", id, log);
    success &= dxl_wb.itemWrite(id, "Feedforward_1st_Gain", f1_gain, &log);
    if (!success) ROS_ERROR("ID %d - Failed to write Feedforward_1st Gain: %s", id, log);
    success &= dxl_wb.itemWrite(id, "Feedforward_2nd_Gain", f2_gain, &log);
    if (!success) ROS_ERROR("ID %d - Failed to write Feedforward_2nd Gain: %s", id, log);
//    auto op_mode = dxl_wb.itemRead(id, "Operating_Mode");
    //cout << "Operating Mode: " << op_mode << endl;

    if (success)
      ROS_INFO("Successfully set PID for ID %d: P=%d I=%d D=%d", id, p_gain, i_gain, d_gain);
    else
      ROS_ERROR("Failed to fully set PID for ID %d", id);
  }

  ROS_INFO("Finished setting all PID gains. Exiting.");
  return 0;
}

