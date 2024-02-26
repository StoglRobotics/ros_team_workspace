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

#include "test_dummy_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using dummy_package_namespace::control_mode_type;
using dummy_package_namespace::NR_CMD_ITFS;
using dummy_package_namespace::NR_STATE_ITFS;

class DummyClassNameTest : public DummyClassNameFixture<TestableDummyClassName>
{
};

TEST_F(DummyClassNameTest, when_controller_is_configured_expect_all_parameters_set)
{
  SetUpController();

  ASSERT_TRUE(controller_->params_.command_joint_names.empty());
  ASSERT_TRUE(controller_->params_.state_joint_names.empty());
  ASSERT_TRUE(controller_->params_.interface_name.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(
    controller_->params_.command_joint_names, testing::ElementsAreArray(command_joint_names_));
  ASSERT_THAT(
    controller_->params_.state_joint_names, testing::ElementsAreArray(state_joint_names_));
  ASSERT_THAT(controller_->state_joint_names_, testing::ElementsAreArray(state_joint_names_));
  ASSERT_EQ(controller_->params_.interface_name, interface_name_);
}

TEST_F(DummyClassNameTest, when_controller_configured_expect_properly_exported_interfaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), joint_command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(command_intefaces.names[i], command_joint_names_[i] + "/" + interface_name_);
  }

  auto state_intefaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), joint_state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(state_intefaces.names[i], state_joint_names_[i] + "/" + interface_name_);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
