$LICENSE$


#ifndef $PACKAGE_NAME$__$FILE_NAME$_HPP_
#define $PACKAGE_NAME$__$FILE_NAME$_HPP_

#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/$interface_type$_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "$package_name$/visibility_control.h"

using hardware_interface::return_type;

namespace $package_name$
{
class $ClassName$ : public
  hardware_interface::BaseInterface<hardware_interface::$Interface_Type$Interface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS($ClassName$);

  $PACKAGE_NAME$_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  $PACKAGE_NAME$_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  $PACKAGE_NAME$_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  $PACKAGE_NAME$_PUBLIC
  return_type start() override;

  $PACKAGE_NAME$_PUBLIC
  return_type stop() override;

  $PACKAGE_NAME$_PUBLIC
  return_type read() override;

  $PACKAGE_NAME$_PUBLIC
  return_type write() override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace $package_name$

#endif  // $PACKAGE_NAME$__$FILE_NAME$_HPP_
