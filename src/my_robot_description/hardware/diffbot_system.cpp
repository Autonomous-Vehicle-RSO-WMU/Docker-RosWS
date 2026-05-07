// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "my_robot_description/diffbot_system.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_description
{
hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.x", &imu_data_.quarternion[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.y", &imu_data_.quarternion[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.z", &imu_data_.quarternion[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.w", &imu_data_.quarternion[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.x", &imu_data_.angular_velocity[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.y", &imu_data_.angular_velocity[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.z", &imu_data_.angular_velocity[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.x", &imu_data_.linear_acceleration[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.y", &imu_data_.linear_acceleration[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.z", &imu_data_.linear_acceleration[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "magnetic_field.x", &imu_data_.magnetic_field[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "magnetic_field.y", &imu_data_.magnetic_field[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "magnetic_field.z", &imu_data_.magnetic_field[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "euler_angles.roll", &imu_data_.euler_angles[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "euler_angles.pitch", &imu_data_.euler_angles[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "euler_angles.yaw", &imu_data_.euler_angles[2]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
 double pos_prev_l = wheel_l_.pos;
  double pos_prev_r = wheel_r_.pos;
  comms_.read_sensor_values(wheel_l_.pos, wheel_r_.pos, imu_data_.linear_acceleration, imu_data_.angular_velocity, imu_data_.magnetic_field);
  // normalize accelerometer
double norm_a = std::sqrt(imu_data_.linear_acceleration[0]*imu_data_.linear_acceleration[0] + imu_data_.linear_acceleration[1]*imu_data_.linear_acceleration[1] + imu_data_.linear_acceleration[2]*imu_data_.linear_acceleration[2]);
if (norm_a != 0.0) { imu_data_.linear_acceleration[0] /= norm_a; imu_data_.linear_acceleration[1] /= norm_a; imu_data_.linear_acceleration[2] /= norm_a; }

double pitch = std::asin(-imu_data_.linear_acceleration[0]);
double roll  = std::atan2(imu_data_.linear_acceleration[1], imu_data_.linear_acceleration[2]);

double my_1 = imu_data_.magnetic_field[1] * std::cos(roll)  - imu_data_.magnetic_field[2] * std::sin(roll);
double mz_1 = imu_data_.magnetic_field[1] * std::sin(roll)  + imu_data_.magnetic_field[2] * std::cos(roll);
double mx_1 = imu_data_.magnetic_field[0] * std::cos(pitch) + mz_1 * std::sin(pitch);

double yaw = std::atan2(-my_1, mx_1);
imu_data_.euler_angles[0] = roll;
imu_data_.euler_angles[1] = pitch;
imu_data_.euler_angles[2] = yaw;
tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);
imu_data_.quarternion[0] = q.x();
imu_data_.quarternion[1] = q.y();
imu_data_.quarternion[2] = q.z();
imu_data_.quarternion[3] = q.w();

  
  
  

  double delta_seconds = period.seconds();

  wheel_l_.vel = (wheel_l_.pos - pos_prev_l) / delta_seconds;

  
  wheel_r_.vel = (wheel_r_.pos - pos_prev_r) / delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type my_robot_description ::DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
  return hardware_interface::return_type::OK;
}

}  // namespace my_robot_description

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  my_robot_description::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
