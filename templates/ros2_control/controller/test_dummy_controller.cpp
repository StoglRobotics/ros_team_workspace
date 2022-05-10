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

#include "test_dummy_controller.hpp"

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

// When there are many mandatory parameters, set all by default and remove one by one in a
// parameterized test
TEST_P(DummyClassNameTestParameterizedParameters, one_parameter_is_missing)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

// TODO(anyone): the new gtest version after 1.8.0 uses INSTANTIATE_TEST_SUITE_P
INSTANTIATE_TEST_SUITE_P(
  MissingMandatoryParameterDuringConfiguration, DummyClassNameTestParameterizedParameters,
  ::testing::Values(
    std::make_tuple(std::string("joints"), rclcpp::ParameterValue(std::vector<std::string>({}))),
    std::make_tuple(
      std::string("state_joints"), rclcpp::ParameterValue(std::vector<std::string>({}))),
    std::make_tuple(std::string("interface_name"), rclcpp::ParameterValue(""))));

TEST_F(DummyClassNameTest, joint_names_parameter_not_set)
{
  SetUpController(false);

  ASSERT_TRUE(controller_->joint_names_.empty());
  ASSERT_TRUE(controller_->state_joint_names_.empty());
  ASSERT_TRUE(controller_->interface_name_.empty());

  controller_->get_node()->set_parameter({"state_joints", state_joint_names_});
  controller_->get_node()->set_parameter({"interface_name", interface_name_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);

  ASSERT_TRUE(controller_->joint_names_.empty());
  ASSERT_TRUE(controller_->state_joint_names_.empty());
  ASSERT_TRUE(controller_->interface_name_.empty());
}

TEST_F(DummyClassNameTest, state_joint_names_parameter_not_set)
{
  SetUpController(false);

  ASSERT_TRUE(controller_->joint_names_.empty());
  ASSERT_TRUE(controller_->state_joint_names_.empty());
  ASSERT_TRUE(controller_->interface_name_.empty());

  controller_->get_node()->set_parameter({"joints", joint_names_});
  controller_->get_node()->set_parameter({"interface_name", interface_name_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);

  ASSERT_THAT(controller_->joint_names_, testing::ElementsAreArray(joint_names_));
  ASSERT_TRUE(controller_->state_joint_names_.empty());
  ASSERT_TRUE(controller_->interface_name_.empty());
}

TEST_F(DummyClassNameTest, interface_parameter_not_set)
{
  SetUpController(false);

  ASSERT_TRUE(controller_->joint_names_.empty());
  ASSERT_TRUE(controller_->state_joint_names_.empty());
  ASSERT_TRUE(controller_->interface_name_.empty());

  controller_->get_node()->set_parameter({"joints", joint_names_});
  controller_->get_node()->set_parameter({"state_joints", state_joint_names_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);

  ASSERT_THAT(controller_->joint_names_, testing::ElementsAreArray(joint_names_));
  ASSERT_THAT(controller_->state_joint_names_, testing::ElementsAreArray(state_joint_names_));
  ASSERT_TRUE(controller_->interface_name_.empty());
}

TEST_F(DummyClassNameTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(controller_->joint_names_.empty());
  ASSERT_TRUE(controller_->state_joint_names_.empty());
  ASSERT_TRUE(controller_->interface_name_.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(controller_->joint_names_, testing::ElementsAreArray(joint_names_));
  ASSERT_THAT(controller_->state_joint_names_, testing::ElementsAreArray(state_joint_names_));
  ASSERT_EQ(controller_->interface_name_, interface_name_);
}

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
    EXPECT_EQ(state_intefaces.names[i], state_joint_names_[i] + "/" + interface_name_);
  }
}

TEST_F(DummyClassNameTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check that the message is reset
  auto msg = controller_->input_cmd_.readFromNonRT();
  EXPECT_EQ((*msg)->displacements.size(), joint_names_.size());
  for (const auto & cmd : (*msg)->displacements) {
    EXPECT_TRUE(std::isnan(cmd));
  }
  EXPECT_EQ((*msg)->velocities.size(), joint_names_.size());
  for (const auto & cmd : (*msg)->velocities) {
    EXPECT_TRUE(std::isnan(cmd));
  }

  ASSERT_TRUE(std::isnan((*msg)->duration));
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
  //   SetUpController();

  ASSERT_EQ(
    controller_->init("test_dummy_package_namespace"), controller_interface::return_type::OK);

  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);

  std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
  command_itfs_.reserve(joint_command_values_.size());
  command_ifs.reserve(joint_command_values_.size());

  for (size_t i = 0; i < joint_command_values_.size(); ++i) {
    command_itfs_.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], interface_name_, &joint_command_values_[i]));
    command_ifs.emplace_back(command_itfs_.back());
  }
  // TODO(anyone): Add other command interfaces, if any

  std::vector<hardware_interface::LoanedStateInterface> state_ifs;
  state_itfs_.reserve(joint_state_values_.size());
  state_ifs.reserve(joint_state_values_.size());

  for (size_t i = 0; i < joint_state_values_.size(); ++i) {
    state_itfs_.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], interface_name_, &joint_state_values_[i]));
    state_ifs.emplace_back(state_itfs_.back());
  }
  // TODO(anyone): Add other state interfaces, if any

  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  controller_->get_node()->set_parameter({"joints", joint_names_});
  controller_->get_node()->set_parameter({"state_joints", state_joint_names_});
  controller_->get_node()->set_parameter({"interface_name", interface_name_});

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
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // set command statically
  static constexpr double TEST_DISPLACEMENT = 23.24;
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_cmd_.writeFromNonRT(msg);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
  EXPECT_EQ(joint_command_values_[STATE_MY_ITFS], TEST_DISPLACEMENT);
  EXPECT_TRUE(std::isnan((*(controller_->input_cmd_.readFromRT()))->displacements[0]));
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::FAST);
}

TEST_F(DummyClassNameTest, test_update_logic_slow)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // set command statically
  static constexpr double TEST_DISPLACEMENT = 23.24;
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  // When slow mode is enabled command ends up being half of the value
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), TEST_DISPLACEMENT);
  msg->velocities.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  controller_->input_cmd_.writeFromNonRT(msg);
  controller_->control_mode_.writeFromNonRT(control_mode_type::SLOW);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SLOW);
  ASSERT_EQ((*(controller_->input_cmd_.readFromRT()))->displacements[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(joint_command_values_[STATE_MY_ITFS], TEST_DISPLACEMENT / 2);
  EXPECT_TRUE(std::isnan((*(controller_->input_cmd_.readFromRT()))->displacements[0]));
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

  publish_commands();
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(joint_command_values_[CMD_MY_ITFS], 0.45);

  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, 0.45);
}
