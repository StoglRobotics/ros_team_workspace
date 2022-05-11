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

#include "dummy_package_namespace/dummy_chainable_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};
}  // namespace

namespace dummy_package_namespace
{
DummyClassName::DummyClassName() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn DummyClassName::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try {
    get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>({}));
    get_node()->declare_parameter<std::vector<std::string>>(
      "state_joints", std::vector<std::string>({}));
    get_node()->declare_parameter<std::string>("interface_name", "");
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DummyClassName::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto error_if_empty = [&](const auto & parameter, const char * parameter_name) {
    if (parameter.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty", parameter_name);
      return true;
    }
    return false;
  };

  auto get_string_array_param_and_error_if_empty =
    [&](std::vector<std::string> & parameter, const char * parameter_name) {
      parameter = get_node()->get_parameter(parameter_name).as_string_array();
      return error_if_empty(parameter, parameter_name);
    };

  auto get_string_param_and_error_if_empty =
    [&](std::string & parameter, const char * parameter_name) {
      parameter = get_node()->get_parameter(parameter_name).as_string();
      return error_if_empty(parameter, parameter_name);
    };

  if (
    get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
    get_string_array_param_and_error_if_empty(state_joint_names_, "state_joints") ||
    get_string_param_and_error_if_empty(interface_name_, "interface_name")) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Command Subscriber and callbacks
  auto callback_cmd = [&](const std::shared_ptr<ControllerCommandMsg> msg) -> void {
    if (msg->joint_names.size() == joint_names_.size()) {
      input_cmd_.writeFromNonRT(msg);
    } else {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Received %zu , but expected %zu joints in command. Ignoring message.",
        msg->joint_names.size(), joint_names_.size());
    }
  };
  cmd_subscriber_ = get_node()->create_subscription<ControllerCommandMsg>(
    "~/commands", rclcpp::SystemDefaultsQoS(), callback_cmd);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response) {
      if (request->data) {
        control_mode_.writeFromNonRT(control_mode_type::SLOW);
      } else {
        control_mode_.writeFromNonRT(control_mode_type::FAST);
      }
      response->success = true;
    };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

    // This fixes the "flaky"-tests issue where publisher is not ready,but we are expecting msg
    // check discussion here: https://github.com/ros-controls/ros2_controllers/pull/327
    while (!state_publisher_->trylock()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    state_publisher_->unlock();
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = joint_names_[0];
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DummyClassName::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(joint_names_.size());
  for (const auto & joint : joint_names_) {
    command_interfaces_config.names.push_back(joint + "/" + interface_name_);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration DummyClassName::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joint_names_.size());
  for (const auto & joint : state_joint_names_) {
    state_interfaces_config.names.push_back(joint + "/" + interface_name_);
  }

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface> DummyClassName::on_export_reference_interfaces()
{
  reference_interfaces_.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  for (size_t i = 0; i < reference_interfaces_.size(); ++i) {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name(), joint_names_[i] + "/" + interface_name_, &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

bool DummyClassName::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn DummyClassName::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  input_cmd_.writeFromNonRT(msg);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DummyClassName::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DummyClassName::update_reference_from_subscribers()
{
  auto current_cmd = input_cmd_.readFromRT();

  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < reference_interfaces_.size(); ++i) {
    if (!std::isnan((*current_cmd)->displacements[i])) {
      reference_interfaces_[i] = (*current_cmd)->displacements[i];

      (*current_cmd)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type DummyClassName::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (!std::isnan(reference_interfaces_[i])) {
      if (*(control_mode_.readFromRT()) == control_mode_type::SLOW) {
        reference_interfaces_[i] /= 2;
      }
      command_interfaces_[i].set_value(reference_interfaces_[i]);

      reference_interfaces_[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();

    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace dummy_package_namespace

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dummy_package_namespace::DummyClassName, controller_interface::ControllerInterface)
