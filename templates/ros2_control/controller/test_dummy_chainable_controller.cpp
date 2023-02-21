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

#include "test_dummy_chainable_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using dummy_package_namespace::control_mode_type;
using dummy_package_namespace::NR_CMD_ITFS;
using dummy_package_namespace::NR_REF_ITFS;
using dummy_package_namespace::NR_STATE_ITFS;

class DummyClassNameTest : public DummyClassNameFixture<TestableDummyClassName>
{
};



// when assigned wrong num of joints then expect in-equality between set values and storage
TEST_F(DummyClassNameTest, when_invalid_reference_msg_is_set_expect_)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.readFromNonRT();
  auto old_timestamp = (*reference)->header.stamp;
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names.size(), joint_names_.size());
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names[0], joint_names_[0]);
  EXPECT_TRUE(std::isnan((*reference)->displacements[0]));
  EXPECT_TRUE(std::isnan((*reference)->velocities[0]));
  EXPECT_TRUE(std::isnan((*reference)->duration));
  publish_commands(controller_->get_node()->now(), {"joint1", "joint2"});
  ASSERT_TRUE(controller_->wait_for_commands(executor));
  EXPECT_NE((*(controller_->input_ref_.readFromNonRT()))->joint_names.size(), joint_names_.size());
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names[0], joint_names_[0]);
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->displacements[0]));
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->velocities[0]));
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->duration));
}

// when age_of_last_command < ref_timeout expect reference msg is accepted and is in rt buffer
TEST_F(DummyClassNameTest, when_message_accepted_expect_reference_msg_in_rt_buffer)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // try to set command with time before timeout - command is not updated
  auto reference = controller_->input_ref_.readFromNonRT();
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names.size(), joint_names_.size());
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names[0], joint_names_[0]);
  EXPECT_TRUE(std::isnan((*reference)->displacements[0]));
  EXPECT_TRUE(std::isnan((*reference)->velocities[0]));
  EXPECT_TRUE(std::isnan((*reference)->duration));
  publish_commands(controller_->get_node()->now());
  ASSERT_TRUE(controller_->wait_for_commands(executor));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names.size(), joint_names_.size());
  EXPECT_NE((*(controller_->input_ref_.readFromNonRT()))->joint_names[0], joint_names_[0]);
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->displacements[0]));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->velocities[0]));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->duration));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->displacements[0], 0.45);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->velocities[0], 0.0);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->duration, 1.25);
}

// when in chainable mode and age_of_last_command < reference_timeout expect
// reference_interfaces set by preceding controller and command_interfaces
// are calculated to non-nan values and reference_interfaces are set to nan
TEST_F(DummyClassNameTest, test_update_logic_chainable_mode)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(true);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }

  // set command statically
  joint_command_values_[0] = 111;

  for (size_t i = 0; i < controller_->reference_interfaces_.size(); ++i) {
    controller_->reference_interfaces_[0] = 1.5;
  }

  const auto age_of_last_command =
    controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;

  // age_of_last_command < ref_timeout_
  ASSERT_TRUE(age_of_last_command <= controller_->ref_timeout_);

  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_NE(joint_command_values_[0], 111);
  // logic is command_interfaces_[i] = reference_interfaces_[i];
  EXPECT_EQ(joint_command_values_[0], 1.5);
  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }
  for (size_t i = 0; i < controller_->command_interfaces_.size(); ++i) {
    EXPECT_EQ(controller_->command_interfaces_[i].get_value(), 1.5);
  }
}

// when ref_timeout = 0 expect reference_msg is accepted and command_interfaces
// are calculated to non-nan values and reference_interfaces are set to nan
TEST_F(DummyClassNameTest, test_ref_timeout_zero_for_update)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.readFromNonRT();

  // set command statically
  static constexpr double TEST_DISPLACEMENT = 23.24;
  controller_->ref_timeout_ = rclcpp::Duration::from_seconds(0.0);
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  msg->header.stamp = controller_->get_node()->now() - rclcpp::Duration::from_seconds(0.0);
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg);
  const auto age_of_last_command =
    controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;

  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(joint_command_values_[NR_STATE_ITFS], TEST_DISPLACEMENT);
  ASSERT_NE((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_TRUE(std::isnan((*(controller_->input_ref_.readFromRT()))->displacements[0]));
}

// when ref_timeout = 0 expect reference_callback() writes reference_msg to rt buffer
// from nonrt thread
TEST_F(
  DummyClassNameTest,
  when_ref_timeout_zero_for_reference_callback_expect_reference_msg_in_rt_buffer)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names.size(), joint_names_.size());
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names[0], joint_names_[0]);
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->displacements[0]));
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->velocities[0]));
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->duration));
  controller_->ref_timeout_ = rclcpp::Duration::from_seconds(0.0);
  //reference_callback() is called implicitly when publish_commands() is called.
  publish_commands(controller_->get_node()->now());
  ASSERT_TRUE(controller_->wait_for_commands(executor));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names.size(), joint_names_.size());
  EXPECT_NE((*(controller_->input_ref_.readFromNonRT()))->joint_names[0], joint_names_[0]);
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->displacements[0]));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->velocities[0]));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->duration));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->displacements[0], 0.45);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->velocities[0], 0.0);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->duration, 1.25);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
