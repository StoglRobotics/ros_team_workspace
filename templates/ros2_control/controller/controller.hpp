$LICENSE$


#ifndef $PACKAGE_NAME$__$FILE_NAME$_HPP_
#define $PACKAGE_NAME$__$FILE_NAME$_HPP_

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"


#include "$package_name$/visibility_control.h"

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
  std::vector<std::string> interface_names_;
};

}  // namespace $package_name$

#endif  // $PACKAGE_NAME$__$FILE_NAME$_HPP_
