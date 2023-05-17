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

using dummy_package_namespace::NR_CMD_ITFS;
using dummy_package_namespace::NR_REF_ITFS;
using dummy_package_namespace::NR_STATE_ITFS;

using dummy_package_namespace::control_mode_type;

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

  // check ref itfs
  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), NR_REF_ITFS);
  for (size_t i = 0; i < NR_REF_ITFS; ++i)
  {
    const std::string ref_itf_name = std::string(controller_->get_node()->get_name()) + "/" +
                                     state_joint_names_[i] + "/" + interface_name_;
    EXPECT_EQ(reference_interfaces[i].get_name(), ref_itf_name);
    EXPECT_EQ(reference_interfaces[i].get_prefix_name(), controller_->get_node()->get_name());
    EXPECT_EQ(
      reference_interfaces[i].get_interface_name(), state_joint_names_[i] + "/" + interface_name_);
  }
}

// when assigned wrong num of joints then expect in-equality between set values and storage
TEST_F(DummyClassNameTest, when_invalid_reference_msg_is_set_expect_reference_reset)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.readFromNonRT();
  auto old_timestamp = (*reference)->header.stamp;
  EXPECT_EQ((*(reference))->joint_names.size(), state_joint_names_.size());
  EXPECT_EQ((*(reference))->joint_names[0], state_joint_names_[0]);
  EXPECT_TRUE(std::isnan((*reference)->displacements[0]));
  EXPECT_TRUE(std::isnan((*reference)->velocities[0]));
  EXPECT_TRUE(std::isnan((*reference)->duration));
  publish_commands(controller_->get_node()->now(), {TEST_DISPLACEMENT}, {"joint1", "joint2"});
  ASSERT_TRUE(controller_->wait_for_commands(executor));
  reference = controller_->input_ref_.readFromNonRT();
  EXPECT_EQ((*(reference))->joint_names.size(), state_joint_names_.size());
  EXPECT_EQ((*(reference))->joint_names[0], state_joint_names_[0]);  
  EXPECT_TRUE(std::isnan((*(reference))->displacements[0]));
  EXPECT_TRUE(std::isnan((*(reference))->velocities[0]));
  EXPECT_TRUE(std::isnan((*(reference))->duration));
}

TEST_F(DummyClassNameTest, when_controller_is_activated_expect_reference_reset)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check that the message is reset
  auto msg = controller_->input_ref_.readFromNonRT();
  EXPECT_EQ((*msg)->displacements.size(), command_joint_names_.size());
  for (const auto & cmd : (*msg)->displacements)
  {
    EXPECT_TRUE(std::isnan(cmd));
  }
  EXPECT_EQ((*msg)->velocities.size(), command_joint_names_.size());
  for (const auto & cmd : (*msg)->velocities)
  {
    EXPECT_TRUE(std::isnan(cmd));
  }

  ASSERT_TRUE(std::isnan((*msg)->duration));

  EXPECT_EQ(controller_->reference_interfaces_.size(), command_joint_names_.size());
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(DummyClassNameTest, when_controller_active_and_update_called_expect_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(DummyClassNameTest, when_active_controller_is_deactivated_expect_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(
  DummyClassNameTest, when_controller_is_reactivated_expect_cmd_itfs_not_set_and_update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->command_interfaces_[NR_CMD_ITFS - 1].get_value(), 101.101);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[NR_CMD_ITFS - 1].get_value()));
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[NR_CMD_ITFS - 1].get_value()));

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

// when update is called expect the previously set reference before calling update,
// inside the controller state message
TEST_F(DummyClassNameTest, when_update_is_called_expect_status_message)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  controller_->reference_interfaces_[NR_STATE_ITFS - 1] = 1.5;

  ControllerStateMsg msg;
  subscribe_to_controller_status_execute_update_and_get_messages(msg);

  EXPECT_EQ(msg.set_point, 1.5);
}

TEST_F(
  DummyClassNameTest,
  when_controller_is_configured_and_activated_properly_expect_correct_setting_of_mode_service)
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

// reference_interfaces and command_interfaces values depend on the reference_msg,
// the below test shows two cases when reference_msg is not received and when it is received.
TEST_F(DummyClassNameTest, when_reference_msg_received_expect_updated_commands_and_status_message)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  // no reference_msg sent
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ControllerStateMsg msg;
  subscribe_to_controller_status_execute_update_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, 0.0);
  joint_command_values_[0] = TEST_DISPLACEMENT;

  // reference_callback() is implicitly called when publish_commands() is called
  // reference_msg is published with provided time stamp when publish_commands( time_stamp)
  // is called
  publish_commands(controller_->get_node()->now());
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // EXPLAIN WHY THE BELOW EQUALITY IS VALID
  // depends on how command_interface values are set inside update methods
  EXPECT_EQ(joint_command_values_[0], 0.45);

  subscribe_to_controller_status_execute_update_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, 0.45);
}

TEST_F(DummyClassNameTest, when_controller_mode_set_fast_expect_update_logic_for_fast_mode)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  msg->joint_names = command_joint_names_;
  msg->displacements.resize(command_joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(command_joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg);

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  EXPECT_EQ(joint_command_values_[NR_STATE_ITFS - 1], TEST_DISPLACEMENT);
  EXPECT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  EXPECT_EQ(controller_->reference_interfaces_.size(), command_joint_names_.size());
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(DummyClassNameTest, when_controller_mode_set_slow_expect_update_logic_for_slow_mode)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  // When slow mode is enabled command ends up being half of the value
  msg->joint_names = command_joint_names_;
  msg->displacements.resize(command_joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(command_joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_.writeFromNonRT(msg);
  controller_->control_mode_.writeFromNonRT(control_mode_type::SLOW);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SLOW);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(joint_command_values_[NR_STATE_ITFS - 1], TEST_DISPLACEMENT / 2);
  EXPECT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  EXPECT_EQ(controller_->reference_interfaces_.size(), command_joint_names_.size());

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(
  DummyClassNameTest,
  when_controller_mode_set_chainable_and_fast_expect_receiving_commands_from_reference_interfaces_directly_with_fast_mode_logic_effect)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(true);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  // set mode to fast
  controller_->control_mode_.writeFromNonRT(control_mode_type::FAST);
  // this is input source in chained mode
  controller_->reference_interfaces_[NR_STATE_ITFS - 1] = TEST_DISPLACEMENT * 2;

  // reference_callback() is implicitly called when publish_commands() is called
  // reference_msg is published with provided time stamp when publish_commands( time_stamp)
  // is called
  publish_commands(controller_->get_node()->now(), {TEST_DISPLACEMENT});
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  // reference_interfaces is directly applied
  EXPECT_EQ(joint_command_values_[NR_STATE_ITFS - 1], TEST_DISPLACEMENT * 2);
  // message is not touched in chained mode
  EXPECT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  EXPECT_EQ(controller_->reference_interfaces_.size(), command_joint_names_.size());
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(
  DummyClassNameTest,
  when_controller_mode_set_chainable_and_slow_expect_receiving_commands_from_reference_interfaces_directly_with_slow_mode_logic_effect)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(true);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  // set mode to slow
  controller_->control_mode_.writeFromNonRT(control_mode_type::SLOW);
  // this is input source in chained mode
  controller_->reference_interfaces_[NR_STATE_ITFS - 1] = TEST_DISPLACEMENT * 4;
  // reference_callback() is implicitly called when publish_commands() is called
  // reference_msg is published with provided time stamp when publish_commands( time_stamp)
  // is called
  publish_commands(controller_->get_node()->now(), {TEST_DISPLACEMENT});
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SLOW);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // reference_interfaces is directly applied / expected slow mode logic effect
  EXPECT_EQ(joint_command_values_[NR_STATE_ITFS - 1], TEST_DISPLACEMENT * 2);
  // message is not touched in chained mode
  EXPECT_EQ((*(controller_->input_ref_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  EXPECT_EQ(controller_->reference_interfaces_.size(), command_joint_names_.size());
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
}

// when time stamp is zero expect that time stamp is set to current time stamp
TEST_F(
  DummyClassNameTest,
  when_reference_msg_has_timestamp_zero_expect_reference_set_and_timestamp_set_to_current_time)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.readFromNonRT();
  auto old_timestamp = (*reference)->header.stamp;
  EXPECT_TRUE(std::isnan((*reference)->displacements[0]));

  // reference_callback() is implicitly called when publish_commands() is called
  // reference_msg is published with provided time stamp when publish_commands( time_stamp)
  // is called
  publish_commands(controller_->get_node()->now());

  ASSERT_TRUE(controller_->wait_for_commands(executor));
  reference = controller_->input_ref_.readFromNonRT();
  ASSERT_EQ(old_timestamp.sec, (*(reference))->header.stamp.sec);
  EXPECT_FALSE(std::isnan((*(reference))->displacements[0]));
  EXPECT_EQ((*(reference))->displacements[0], 0.45);

  EXPECT_NE((*(reference))->header.stamp.sec, 0.0);
}

// when the reference_msg has valid timestamp then the timeout check in reference_callback()
// shall pass and reference_msg is not reset
TEST_F(DummyClassNameTest, when_message_has_valid_timestamp_expect_reference_set)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.readFromNonRT();
  EXPECT_TRUE(std::isnan((*reference)->displacements[0]));

  // reference_callback() is implicitly called when publish_commands() is called
  // reference_msg is published with provided time stamp when publish_commands( time_stamp)
  // is called
  publish_commands(controller_->get_node()->now());

  ASSERT_TRUE(controller_->wait_for_commands(executor));
  reference = controller_->input_ref_.readFromNonRT();
  EXPECT_FALSE(std::isnan((*(reference))->displacements[0]));
  EXPECT_EQ((*(reference))->displacements[0], 0.45);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
