$LICENSE$

#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class Test$ClassName$ : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // TODO(anyone): Extend this description to your robot
    $file_name$_2dof_ =
        R"(
  <ros2_control name="$ClassName$2dof" type="$interface_type$">
    <hardware>
      <plugin>$package_name$/$ClassName$</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="initial_position">1.57</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="initial_position">0.7854</param>
    </joint>
  </ros2_control>
)";
  }

  std::string $file_name$_2dof_;
};

TEST_F(Test$ClassName$, load_$file_name$_2dof)
{
  auto urdf =
      ros2_control_test_assets::urdf_head + $file_name$_2dof_ + ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
