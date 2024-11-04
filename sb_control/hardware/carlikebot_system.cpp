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

#include "ros2_control_demo_example_11/carlikebot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_11
{
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CarlikeBotSystemHardware"),
      "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production
  // code
  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  hw_interfaces_["steering_rack"] = Joint("steering_rack_joint");
  hw_interfaces_["virtual_rear_wheel"] = Joint("virtual_rear_wheel_joint");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    hw_interfaces_["steering_rack"].joint_name, hardware_interface::HW_IF_POSITION, 
    &hw_interfaces_["steering_rack"].state.position));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    hw_interfaces_["virtual_rear_wheel"].joint_name, hardware_interface::HW_IF_POSITION, 
    &hw_interfaces_["virtual_rear_wheel"].state.position));

  // Для задних колес - интерфейс скорости
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    hw_interfaces_["virtual_rear_wheel"].joint_name, hardware_interface::HW_IF_VELOCITY, 
    &hw_interfaces_["virtual_rear_wheel"].state.velocity));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CarlikeBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    hw_interfaces_["steering_rack"].joint_name, hardware_interface::HW_IF_POSITION,
    &hw_interfaces_["steering_rack"].command.position));

  // Команды для задних колес (скорость) - через интерфейс скорости
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    hw_interfaces_["virtual_rear_wheel"].joint_name, hardware_interface::HW_IF_VELOCITY,
    &hw_interfaces_["virtual_rear_wheel"].command.velocity));

  return command_interfaces;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("CarlikeBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }

  

  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("CarlikeBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarlikeBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

  // FIXME: deal with the source of NaNs here
  if (std::isnan(hw_interfaces_["virtual_rear_wheel"].command.velocity)) {
    return hardware_interface::return_type::OK;
  }
  hw_interfaces_["steering_rack"].state.position = hw_interfaces_["steering_rack"].command.position;
  hw_interfaces_["virtual_rear_wheel"].state.velocity = hw_interfaces_["virtual_rear_wheel"].command.velocity;
  hw_interfaces_["virtual_rear_wheel"].state.position += hw_interfaces_["virtual_rear_wheel"].state.velocity * period.seconds();
 
  RCLCPP_INFO(
    rclcpp::get_logger("CarlikeBotSystemHardware"), "Got position state: %.2f for joint '%s'.", 
    hw_interfaces_["steering_rack"].command.position, hw_interfaces_["steering_rack"].joint_name.c_str());
    
  RCLCPP_INFO(
    rclcpp::get_logger("CarlikeBotSystemHardware"), "Got velocity state: %.2f for joint '%s'.",
    hw_interfaces_["virtual_rear_wheel"].command.velocity, hw_interfaces_["virtual_rear_wheel"].joint_name.c_str());

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_11 ::CarlikeBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(
    rclcpp::get_logger("CarlikeBotSystemHardware"), "Got position command: %.2f for joint '%s'.",
    hw_interfaces_["steering_rack"].command.position, hw_interfaces_["steering_rack"].joint_name.c_str());

  RCLCPP_INFO(
    rclcpp::get_logger("CarlikeBotSystemHardware"), "Got velocity command: %.2f for joint '%s'.",
    hw_interfaces_["virtual_rear_wheel"].command.velocity, hw_interfaces_["virtual_rear_wheel"].joint_name.c_str());

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_11

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_11::CarlikeBotSystemHardware, hardware_interface::SystemInterface)
