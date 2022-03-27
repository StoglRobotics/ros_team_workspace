$LICENSE$

#ifndef TEMPLATES__ROS2_CONTROL__CONTROLLER__DUMMY_CONTROLLER_HPP_
#define TEMPLATES__ROS2_CONTROL__CONTROLLER__DUMMY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "dummy_package_namespace/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace dummy_package_namespace
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DummyClassName : public controller_interface::ControllerInterface
{
public:
  TEMPLATES__ROS2_CONTROL__CONTROLLER_PUBLIC
  DummyClassName();

  TEMPLATES__ROS2_CONTROL__CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  TEMPLATES__ROS2_CONTROL__CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  TEMPLATES__ROS2_CONTROL__CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  TEMPLATES__ROS2_CONTROL__CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;
  std::string interface_name_;

  // TODO(anyone): replace the state and command message types
  // Command subscribers and Controller State publisher
  using ControllerCommandMsg = control_msgs::msg::JointJog;

  rclcpp::Subscription<ControllerCommandMsg>::SharedPtr command_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandMsg>> input_command_;

  using ControllerStateMsg = control_msgs::msg::JointControllerState;
  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;
};

}  // namespace dummy_package_namespace

#endif  // TEMPLATES__ROS2_CONTROL__CONTROLLER__DUMMY_CONTROLLER_HPP_
