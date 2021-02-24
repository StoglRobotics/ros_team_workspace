$LICENSE$

#include "$package_name$/$file_name$.hpp"

#include <string>
#include <vector>

namespace $package_name$
{

$ClassName$::$ClassName$()
: controller_interface::ControllerInterface()
{}

controller_interface::return_type
$ClassName$::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::SUCCESS) {
    return ret;
  }

  try {
    auto node = get_node();
    node->declare_parameter<std::vector<std::string>>("joints", {});
    node->declare_parameter<std::string>("interface_name", "");
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::SUCCESS;
}

CallbackReturn $ClassName$::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = node_->get_parameter("joints").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::ERROR;
  }

  // Specialized, child controllers set interfaces before calling init function.
  if (interface_names_.empty()) {
    interface_names_ = node_->get_parameter("interface_names").as_string_array();
  }

  if (interface_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
$ClassName$::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_) {
    for (const auto & interface : interface_names_) {
      command_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
$ClassName$::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_) {
    for (const auto & interface : interface_names_) {
      state_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return state_interfaces_config;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template<typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names) {
    for (auto & command_interface : unordered_interfaces) {
      if ((command_interface.get_name() == joint_name) &&
        (command_interface.get_interface_name() == interface_type))
      {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn $ClassName$::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn $ClassName$::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type $ClassName$::update()
{
  for (auto index = 0ul; index < command_interfaces_.size(); ++index) {
    command_interfaces_[index].set_value(state_interfaces_[index].get_value());
  }

  return controller_interface::return_type::SUCCESS;
}

}  // namespace $package_name$

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  $package_name$::$ClassName$,
  controller_interface::ControllerInterface)
