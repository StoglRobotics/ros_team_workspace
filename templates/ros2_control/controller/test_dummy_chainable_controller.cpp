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

using dummy_package_namespace::NR_CMD_ITFS;
using dummy_package_namespace::control_mode_type;
using dummy_package_namespace::NR_STATE_ITFS;
using dummy_package_namespace::NR_REF_ITFS;


class DummyClassNameTest : public DummyClassNameFixture<TestableDummyClassName>
{
};

// when_all_parameters_are_set_expect_them_in_storage

TEST_F(DummyClassNameTest, all_parameters_set_configure_success)
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

// when all command, state and reference interfaces are exported then expect them in storage
TEST_F(DummyClassNameTest, check_exported_intefaces)
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

// when calling activate() expect resetting of the controller reference msg
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

// when calling update methods expect return type are a success
TEST_F(DummyClassNameTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

// when controller lifecycle methods expect return type is a success
TEST_F(DummyClassNameTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

// when calling on_activate, on_deactivate and on_activate methods consecutively 
// expect resetting of reference msg, nan values in command_interfaces and 
// resetting of reference msg respectively
TEST_F(DummyClassNameTest, reactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->command_interfaces_[NR_CMD_ITFS].get_value(), 101.101);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[NR_CMD_ITFS].get_value()));
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[NR_CMD_ITFS].get_value()));

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

// when set slow mode service expect the same in storage
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

// when not in chainable mode,set to FAST mode service and 
// age_of_last_command > reference_timeout expect
// command_interfaces are set to 0.0 and reference_interfaces set to nan
// followed by
// when not in chainable mode and age_of_last_command < reference_timeout expect
// command_interfaces are calculated to non-nan and reference_interfaces set to nan
TEST_F(DummyClassNameTest, test_update_logic_not_chainable_mode_fast)
{// 1. age<ref_timeout 2.age>ref_timeout
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
  joint_command_values_[0] = 111;
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  msg->header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ - rclcpp::Duration::from_seconds(0.1);
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg);
  const auto age_of_last_command = controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);

  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  EXPECT_EQ(joint_command_values_[0], 0.0);
  ASSERT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }

  std::shared_ptr<ControllerReferenceMsg> msg_2 = std::make_shared<ControllerReferenceMsg>();
  msg_2->header.stamp = controller_->get_node()->now() - rclcpp::Duration::from_seconds(0.01);
  msg_2->joint_names = joint_names_;
  msg_2->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg_2->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg_2->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg_2);
  const auto age_of_last_command_2 = controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);

  ASSERT_TRUE(age_of_last_command_2 <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  EXPECT_EQ(joint_command_values_[0], TEST_DISPLACEMENT);//exact value
  EXPECT_NE(joint_command_values_[0], 111);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }
}

// when not in chainable mode,set to SLOW mode service and 
// age_of_last_command > reference_timeout expect
// command_interfaces are set to 0.0 and reference_interfaces set to nan
// followed by
// when not in chainable mode and age_of_last_command < reference_timeout expect
// command_interfaces are calculated to non-nan and reference_interfaces set to nan
TEST_F(DummyClassNameTest, test_update_logic_not_chainable_mode_slow)
{// 1. age<ref_timeout 2.age>ref_timeout
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
  joint_command_values_[0] = 111;
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  msg->header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ - rclcpp::Duration::from_seconds(0.1);
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg);
  const auto age_of_last_command = controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;
  controller_->control_mode_.writeFromNonRT(control_mode_type::SLOW);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SLOW);
  //age_of_last_command > ref_timeout_
  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SLOW);
  EXPECT_EQ(joint_command_values_[0], 0.0);
  ASSERT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }

  std::shared_ptr<ControllerReferenceMsg> msg_2 = std::make_shared<ControllerReferenceMsg>();
  msg_2->header.stamp = controller_->get_node()->now() - rclcpp::Duration::from_seconds(0.01);
  msg_2->joint_names = joint_names_;
  msg_2->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg_2->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg_2->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg_2);
  const auto age_of_last_command_2 = controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;
  //age_of_last_command_2 < ref_timeout_
  ASSERT_TRUE(age_of_last_command_2 <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SLOW);
  EXPECT_EQ(joint_command_values_[0], TEST_DISPLACEMENT/4);//exact value
  EXPECT_NE(joint_command_values_[0], 111);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT/2);
  for (const auto & interface : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(interface));
  }
}

// when controller state published expect state value in storage
TEST_F(DummyClassNameTest, publish_status_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, 101.101);
}

// when msg subscribed and published expect value in storage
TEST_F(DummyClassNameTest, receive_message_and_publish_updated_status)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
//here
  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, 101.101);

  publish_commands(controller_->get_node()->now());
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
//here
  EXPECT_EQ(joint_command_values_[NR_CMD_ITFS], 0.45);

  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, 0.45);
}

// when too old msg is sent expect nan values in reference msg
TEST_F(DummyClassNameTest, test_sending_too_old_message)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = *(controller_->input_ref_.readFromNonRT());
  auto old_timestamp = reference->header.stamp;
  EXPECT_TRUE(std::isnan(reference->displacements[0]));
  EXPECT_TRUE(std::isnan(reference->velocities[0]));
  EXPECT_TRUE(std::isnan(reference->duration));

  // reference_callback() is implicitly called when publish_commands() is called
  publish_commands(
    controller_->get_node()->now() - controller_->ref_timeout_ -
    rclcpp::Duration::from_seconds(0.1));
  ASSERT_TRUE(controller_->wait_for_commands(executor));
  ASSERT_EQ(old_timestamp, (*(controller_->input_ref_.readFromNonRT()))->header.stamp);
  EXPECT_TRUE(std::isnan((reference)->displacements[0]));
  EXPECT_TRUE(std::isnan((reference)->velocities[0]));
  EXPECT_TRUE(std::isnan((reference)->duration));
}

// when time stamp is zero expect that time stamp is set to current time stamp
TEST_F(DummyClassNameTest, test_time_stamp_zero)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.readFromNonRT();
  auto old_timestamp = (*reference)->header.stamp;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    EXPECT_TRUE(std::isnan((*reference)->displacements[i]));
  }
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    EXPECT_TRUE(std::isnan((*reference)->velocities[i]));
  }

  // reference_callback() is implicitly called when publish_commands() is called
  publish_commands(rclcpp::Time(0));

  ASSERT_TRUE(controller_->wait_for_commands(executor));
  ASSERT_EQ(old_timestamp.sec, (*(controller_->input_ref_.readFromNonRT()))->header.stamp.sec);
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->displacements[0]));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->velocities[0]));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->displacements[0], 0.45);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->velocities[0], 0.0);
  EXPECT_NE((*(controller_->input_ref_.readFromNonRT()))->header.stamp.sec, 0.0);
}


// when assigned wrong num of joints then expect in-equality between set values and storage
TEST_F(DummyClassNameTest, test_message_wrong_num_joints)
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
  publish_commands(controller_->get_node()->now(), {"joint1","joint2"});
  ASSERT_TRUE(controller_->wait_for_commands(executor));
  EXPECT_NE((*(controller_->input_ref_.readFromNonRT()))->joint_names.size(), joint_names_.size());
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->joint_names[0], joint_names_[0]);
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->displacements[0]));
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->velocities[0]));
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->duration));
}

// when age_of_last_command < ref_timeout expect reference msg is accepted and is in rt buffer
TEST_F(DummyClassNameTest, test_message_accepted)
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

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  // set command statically
  joint_command_values_[0] = 111;

  for (size_t i = 0; i < controller_->reference_interfaces_.size(); ++i)
  {
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
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  for (size_t i = 0; i < controller_->command_interfaces_.size(); ++i)
  {
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
  controller_->ref_timeout_= rclcpp::Duration::from_seconds(0.0);
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  msg->header.stamp = controller_->get_node()->now() - rclcpp::Duration::from_seconds(0.0);
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg);
  const auto age_of_last_command = controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;
  
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
TEST_F(DummyClassNameTest, test_ref_timeout_zero_for_reference_callback)
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
  controller_->ref_timeout_= rclcpp::Duration::from_seconds(0.0);  
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
