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

#include "test_dummy_chainable_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using dummy_package_namespace::CMD_MY_ITFS;
using dummy_package_namespace::control_mode_type;
using dummy_package_namespace::STATE_MY_ITFS;

class DummyClassNameTest : public DummyClassNameFixture<TestableDummyClassName>
{
};

TEST_F(DummyClassNameTest, when_controller_is_configured_expect_all_parameters_set)
{
  SetUpController();

  ASSERT_TRUE(controller_->params_.joints.empty());
  ASSERT_TRUE(controller_->params_.state_joints.empty());
  ASSERT_TRUE(controller_->params_.interface_name.empty());
  ASSERT_EQ(controller_->params_.reference_timeout, 0.0);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(controller_->params_.joints, testing::ElementsAreArray(joint_names_));
  ASSERT_TRUE(controller_->params_.state_joints.empty());
  ASSERT_THAT(controller_->state_joints_, testing::ElementsAreArray(joint_names_));
  ASSERT_EQ(controller_->params_.interface_name, interface_name_);
  ASSERT_EQ(controller_->params_.reference_timeout, 0.1);
}

TEST_F(DummyClassNameTest, when_controller_configured_expect_properly_exported_interfaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), joint_command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i) {
    EXPECT_EQ(command_intefaces.names[i], joint_names_[i] + "/" + interface_name_);
  }

  auto state_intefaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), joint_state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i) {
    EXPECT_EQ(state_intefaces.names[i], joint_names_[i] + "/" + interface_name_);
  }

  // check ref itfs
  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string ref_itf_name = std::string(controller_->get_node()->get_name()) + "/" +
                                     joint_names_[i] + "/" + interface_name_;
    EXPECT_EQ(reference_interfaces[i].get_name(), ref_itf_name);
    EXPECT_EQ(reference_interfaces[i].get_prefix_name(), controller_->get_node()->get_name());
    EXPECT_EQ(
      reference_interfaces[i].get_interface_name(), joint_names_[i] + "/" + interface_name_);
  }
}

TEST_F(DummyClassNameTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check that the message is reset
  auto msg = controller_->input_ref_.readFromNonRT();
  EXPECT_EQ((*msg)->displacements.size(), joint_names_.size());
  for (const auto & cmd : (*msg)->displacements) {
    EXPECT_TRUE(std::isnan(cmd));
  }
  EXPECT_EQ((*msg)->velocities.size(), joint_names_.size());
  for (const auto & cmd : (*msg)->velocities) {
    EXPECT_TRUE(std::isnan(cmd));
  }

  ASSERT_TRUE(std::isnan((*msg)->duration));

  EXPECT_EQ(controller_->reference_interfaces_.size(), joint_names_.size());
  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(DummyClassNameTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(DummyClassNameTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(DummyClassNameTest, reactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->command_interfaces_[CMD_MY_ITFS].get_value(), 101.101);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[CMD_MY_ITFS].get_value()));
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[CMD_MY_ITFS].get_value()));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(DummyClassNameTest, test_setting_slow_mode_service)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  // initially set to false
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // should stay false
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);

  // set to true
  ASSERT_NO_THROW(call_service(true, executor));
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SLOW);

  // set back to false
  ASSERT_NO_THROW(call_service(false, executor));
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
}

TEST_F(DummyClassNameTest, test_update_logic_fast)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());

  // set command statically
  static constexpr double TEST_DISPLACEMENT = 23.24;
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  msg->header.stamp = controller_->get_node()->now();
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  EXPECT_EQ(joint_command_values_[STATE_MY_ITFS], TEST_DISPLACEMENT);
  EXPECT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);  
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  EXPECT_EQ(controller_->reference_interfaces_.size(), joint_names_.size());
  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(DummyClassNameTest, test_update_logic_slow)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());

  // set command statically
  static constexpr double TEST_DISPLACEMENT = 23.24;
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  // When slow mode is enabled command ends up being half of the value
  msg->header.stamp = controller_->get_node()->now();
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg);
  controller_->control_mode_.writeFromNonRT(control_mode_type::SLOW);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SLOW);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(joint_command_values_[STATE_MY_ITFS], TEST_DISPLACEMENT / 4);
  EXPECT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT/2);  
  EXPECT_EQ(controller_->reference_interfaces_.size(), joint_names_.size());
  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(DummyClassNameTest, test_update_logic_chainable_fast)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(true);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  // set command statically
  static constexpr double TEST_DISPLACEMENT = 23.24;
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  msg->header.stamp = controller_->get_node()->now();
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg);
  // this is input source in chained mode
  controller_->reference_interfaces_[STATE_MY_ITFS] = TEST_DISPLACEMENT * 2;

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  // reference_interfaces is directly applied
  EXPECT_EQ(joint_command_values_[STATE_MY_ITFS], TEST_DISPLACEMENT * 2);
  // message is not touched in chained mode
  EXPECT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  EXPECT_EQ(controller_->reference_interfaces_.size(), joint_names_.size());
  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(DummyClassNameTest, test_update_logic_chainable_slow)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(true);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  // set command statically
  static constexpr double TEST_DISPLACEMENT = 23.24;
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  // When slow mode is enabled command ends up being half of the value
  msg->header.stamp = controller_->get_node()->now();
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg);
  controller_->control_mode_.writeFromNonRT(control_mode_type::SLOW);
  // this is input source in chained mode
  controller_->reference_interfaces_[STATE_MY_ITFS] = TEST_DISPLACEMENT * 4;

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SLOW);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // reference_interfaces is directly applied
  EXPECT_EQ(joint_command_values_[STATE_MY_ITFS], TEST_DISPLACEMENT * 2);
  // message is not touched in chained mode
  EXPECT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  EXPECT_EQ(controller_->reference_interfaces_.size(), joint_names_.size());
  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(DummyClassNameTest, publish_status_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, 101.101);
}

TEST_F(DummyClassNameTest, receive_message_and_publish_updated_status)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, 101.101);

  publish_commands(controller_->get_node()->now());
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(joint_command_values_[CMD_MY_ITFS], 0.45);

  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, 0.45);
}

// when too old msg is sent expect reference msg reset
TEST_F(DummyClassNameTest, when_reference_msg_is_too_old_expect_unset_reference)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = *(controller_->input_ref_.readFromNonRT());
  auto old_timestamp = reference->header.stamp;
  EXPECT_TRUE(std::isnan(reference->displacements[0]));

  // reference_callback() is implicitly called when publish_commands() is called
  // reference_msg is published with provided time stamp when publish_commands( time_stamp)
  // is called
  publish_commands(
    controller_->get_node()->now() - controller_->ref_timeout_ -
    rclcpp::Duration::from_seconds(0.1));
  ASSERT_TRUE(controller_->wait_for_commands(executor));
  ASSERT_EQ(old_timestamp, (*(controller_->input_ref_.readFromNonRT()))->header.stamp);
  EXPECT_TRUE(std::isnan(reference->displacements[0]));
}

// when not in chainable mode and ref_msg_timedout expect
// command_interfaces are set to 0.0 and when ref_msg is not timedout expect
// command_interfaces are set to valid command values
TEST_F(DummyClassNameTest, when_ref_msg_old_expect_cmnd_itfs_set_to_zero_otherwise_to_valid_cmnds)
{
  // 1. age>ref_timeout 2. age<ref_timeout
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());

  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }

  joint_command_values_[0] = TEST_DISPLACEMENT;
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  msg->header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                      rclcpp::Duration::from_seconds(0.1);
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg);
  const auto age_of_last_command =
    controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;

  // age_of_last_command > ref_timeout_
  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(joint_command_values_[0], 0.0);
  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }
  for (size_t i = 0; i < controller_->command_interfaces_.size(); ++i) {
    EXPECT_EQ(controller_->command_interfaces_[i].get_value(), 0.0);
  }

  std::shared_ptr<ControllerReferenceMsg> msg_2 = std::make_shared<ControllerReferenceMsg>();
  msg_2->header.stamp = controller_->get_node()->now() - rclcpp::Duration::from_seconds(0.01);
  msg_2->joint_names = joint_names_;
  msg_2->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg_2->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg_2->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg_2);
  const auto age_of_last_command_2 =
    controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  // age_of_last_command_2 < ref_timeout_
  ASSERT_TRUE(age_of_last_command_2 <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(joint_command_values_[0], TEST_DISPLACEMENT);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }

  for (size_t i = 0; i < controller_->command_interfaces_.size(); ++i) {
    EXPECT_EQ(controller_->command_interfaces_[i].get_value(), TEST_DISPLACEMENT);
  }
}

// when ref_timeout = 0 expect reference_msg is accepted only once and command_interfaces
// are calculated to valid values and reference_interfaces are unset
TEST_F(DummyClassNameTest, when_reference_timeout_is_zero_expect_reference_msg_being_used_only_once)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  joint_command_values_[0] = TEST_DISPLACEMENT;
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

  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_FALSE(std::isnan(joint_command_values_[0]));
  ASSERT_NE((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  EXPECT_EQ(joint_command_values_[STATE_MY_ITFS], TEST_DISPLACEMENT);
  ASSERT_TRUE(std::isnan((*(controller_->input_ref_.readFromRT()))->displacements[0]));
}

TEST_F(
  DummyClassNameTest,
  when_ref_timeout_zero_for_reference_callback_expect_reference_msg_being_used_only_once)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names.size(), joint_names_.size());
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names[0], joint_names_[0]);
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->displacements[0]));
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->velocities[0]));
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->duration));
  controller_->ref_timeout_ = rclcpp::Duration::from_seconds(0.0);
  // reference_callback() is called implicitly when publish_commands() is called.
  publish_commands(controller_->get_node()->now());
  ASSERT_TRUE(controller_->wait_for_commands(executor));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names.size(), joint_names_.size());
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names[0], joint_names_[0]);
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
