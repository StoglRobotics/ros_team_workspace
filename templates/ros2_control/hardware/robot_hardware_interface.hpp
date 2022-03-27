$LICENSE$

#ifndef TEMPLATES__ROS2_CONTROL__HARDWARE__ROBOT_HARDWARE_INTERFACE_HPP_
#define TEMPLATES__ROS2_CONTROL__HARDWARE__ROBOT_HARDWARE_INTERFACE_HPP_

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
class DummyClassName : public hardware_interface::Dummy_Interface_TypeInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DummyClassName);

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

#endif  // TEMPLATES__ROS2_CONTROL__HARDWARE__ROBOT_HARDWARE_INTERFACE_HPP_
