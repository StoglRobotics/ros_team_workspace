// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include "dummy_package_namespace/dummy_controller.hpp"

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

using ControllerReferenceMsg = dummy_package_namespace::DummyClassName::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names,
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  msg->header.stamp = node->now();
  msg->joint_names = joint_names;
  msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace dummy_package_namespace
{
DummyClassName::DummyClassName() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn DummyClassName::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<dummy_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DummyClassName::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.state_joint_names.empty())
  {
    state_joint_names_ = params_.state_joint_names;
  }
  else
  {
    state_joint_names_ = params_.command_joint_names;
  }

  if (params_.command_joint_names.size() != state_joint_names_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'command_joint_names' (%d) and 'state_joint_names' (%d) parameters has to be the "
      "same!",
      params_.command_joint_names.size(), state_joint_names_.size());
    return CallbackReturn::FAILURE;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&DummyClassName::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.command_joint_names, get_node());
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher
    s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = params_.command_joint_names[0];
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void DummyClassName::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  // if no timestamp provided use current time for command timestamp
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
                "Timestamp in header is missing, using current time as command "
                "timestamp.");
    msg->header.stamp = get_node()->now();
  }
  if (msg->joint_names.size() == params_.command_joint_names.size())
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
                 "Received %zu , but expected %zu command_joint_names in command. Ignoring message.",
                 msg->joint_names.size(), params_.command_joint_names.size());
  }
}

controller_interface::InterfaceConfiguration DummyClassName::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.command_joint_names.size());
  for (const auto & joint : params_.command_joint_names)
  {
    command_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration DummyClassName::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joint_names_.size());
  for (const auto & joint : state_joint_names_)
  {
    state_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn DummyClassName::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for examplary use of
  // `controller_interface::get_ordered_interfaces` helper function

  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT()), state_joint_names_, get_node());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DummyClassName::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i <= NR_CMD_ITFS; ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DummyClassName::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();

  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (!std::isnan((*current_ref)->displacements[i]))
    {
      if (*(control_mode_.readFromRT()) == control_mode_type::SLOW)
      {
        (*current_ref)->displacements[i] /= 2;
      }
      command_interfaces_[i].set_value((*current_ref)->displacements[i]);

    }else
    {
        command_interfaces_[i].set_value(0.0);
    }
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[NR_CMD_ITFS].get_value();

    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace dummy_package_namespace

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dummy_package_namespace::DummyClassName, controller_interface::ControllerInterface)
