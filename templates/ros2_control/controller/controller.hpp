$LICENSE$

#ifndef $PACKAGE_NAME$__$FILE_NAME$_HPP_
#define $PACKAGE_NAME$__$FILE_NAME$_HPP_

#include <memory>
#include <string>
#include <vector>

#include "$package_name$/visibility_control.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace $package_name$
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class $ClassName$ : public controller_interface::ControllerInterface
{
public:
  $PACKAGE_NAME$_PUBLIC
  $ClassName$();

  $PACKAGE_NAME$_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  $PACKAGE_NAME$_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  $PACKAGE_NAME$_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  $PACKAGE_NAME$_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  $PACKAGE_NAME$_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  $PACKAGE_NAME$_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  $PACKAGE_NAME$_PUBLIC
  controller_interface::return_type update() override;

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

}  // namespace $package_name$

#endif  // $PACKAGE_NAME$__$FILE_NAME$_HPP_
