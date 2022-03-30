// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef TEMPLATES__ROS2_CONTROL__HARDWARE__DUMMY_PACKAGE_NAMESPACE__ROBOT_HARDWARE_INTERFACE_HPP_
#define TEMPLATES__ROS2_CONTROL__HARDWARE__DUMMY_PACKAGE_NAMESPACE__ROBOT_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "dummy_package_namespace/visibility_control.h"
#include "hardware_interface/dummy_interface_type_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dummy_package_namespace
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DummyClassName : public hardware_interface::Dummy_Interface_TypeInterface
{
public:
  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace dummy_package_namespace

#endif  // TEMPLATES__ROS2_CONTROL__HARDWARE__DUMMY_PACKAGE_NAMESPACE__ROBOT_HARDWARE_INTERFACE_HPP_
