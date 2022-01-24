$LICENSE$

#include <limits>
#include <vector>

#include "$package_name$/$file_name$.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace $package_name$
{
CallbackReturn $ClassName$::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::$Interface_Type$Interface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // TODO(anyone): read parameters and initialize the hardware
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> $ClassName$::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> $ClassName$::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn $ClassName$::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands

  return CallbackReturn::SUCCESS;
}

CallbackReturn $ClassName$::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type $ClassName$::read()
{
  // TODO(anyone): read robot states

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type $ClassName$::write()
{
  // TODO(anyone): write robot's commands'

  return hardware_interface::return_type::OK;
}

}  // namespace $package_name$

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS($package_name$::$ClassName$, hardware_interface::$Interface_Type$Interface)
