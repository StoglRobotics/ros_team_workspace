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

#ifndef TEMPLATES__ROS2_CONTROL__CONTROLLER__TEST_DUMMY_CHAINABLE_CONTROLLER_HPP_
#define TEMPLATES__ROS2_CONTROL__CONTROLLER__TEST_DUMMY_CHAINABLE_CONTROLLER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "dummy_package_namespace/dummy_chainable_controller.hpp"
#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

// TODO(anyone): replace the state and command message types
using ControllerStateMsg = dummy_package_namespace::DummyClassName::ControllerStateMsg;
using ControllerReferenceMsg = dummy_package_namespace::DummyClassName::ControllerReferenceMsg;
using ControllerModeSrvType = dummy_package_namespace::DummyClassName::ControllerModeSrvType;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace
// namespace

// subclassing and friending so we can access member variables
class TestableDummyClassName : public dummy_package_namespace::DummyClassName
{
  FRIEND_TEST(DummyClassNameTest, when_controller_is_configured_expect_all_parameters_set);
  FRIEND_TEST(DummyClassNameTest, when_controller_configured_expect_properly_exported_interfaces);
  FRIEND_TEST(DummyClassNameTest, when_invalid_reference_msg_is_set_expect_reference_reset);
  FRIEND_TEST(DummyClassNameTest, when_controller_is_activated_expect_reference_reset);
  FRIEND_TEST(DummyClassNameTest, when_controller_active_and_update_called_expect_success);
  FRIEND_TEST(DummyClassNameTest, when_active_controller_is_deactivated_expect_success);
  FRIEND_TEST(
    DummyClassNameTest, when_controller_is_reactivated_expect_cmd_itfs_not_set_and_update_success);
  FRIEND_TEST(DummyClassNameTest, when_update_is_called_expect_status_message);
  FRIEND_TEST(
    DummyClassNameTest,
    when_controller_is_configured_and_activated_properly_expect_correct_setting_of_mode_service);
  FRIEND_TEST(
    DummyClassNameTest, when_reference_msg_received_expect_updated_commands_and_status_message);
  FRIEND_TEST(DummyClassNameTest, when_controller_mode_set_fast_expect_update_logic_for_fast_mode);
  FRIEND_TEST(DummyClassNameTest, when_controller_mode_set_slow_expect_update_logic_for_slow_mode);
  FRIEND_TEST(
    DummyClassNameTest,
    when_controller_mode_set_chainable_and_fast_expect_receiving_commands_from_reference_interfaces_directly_with_fast_mode_logic_effect);
  FRIEND_TEST(
    DummyClassNameTest,
    when_controller_mode_set_chainable_and_slow_expect_receiving_commands_from_reference_interfaces_directly_with_slow_mode_logic_effect);
  FRIEND_TEST(
    DummyClassNameTest,
    when_reference_msg_has_timestamp_zero_expect_reference_set_and_timestamp_set_to_current_time);
  FRIEND_TEST(DummyClassNameTest, when_message_has_valid_timestamp_expect_reference_set);
  FRIEND_TEST(DummyClassNameTest, when_loading_controller_expect_no_exception);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = dummy_package_namespace::DummyClassName::on_configure(previous_state);
    // Only if on_configure is successful create subscription
    if (ret == CallbackReturn::SUCCESS)
    {
      ref_subscriber_wait_set_.add_subscription(ref_subscriber_);
    }
    return ret;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ref_itfs = on_export_reference_interfaces();
    return dummy_package_namespace::DummyClassName::on_activate(previous_state);
  }

  /**
   * @brief wait_for_command blocks until a new ControllerReferenceMsg is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function.
   *
   * @return true if new ControllerReferenceMsg msg was received, false if timeout.
   */
  bool wait_for_command(
    rclcpp::Executor & executor, rclcpp::WaitSet & subscriber_wait_set,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    bool success = subscriber_wait_set.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;
    if (success)
    {
      executor.spin_some();
    }
    return success;
  }

  bool wait_for_commands(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    return wait_for_command(executor, ref_subscriber_wait_set_, timeout);
  }

  // TODO(anyone): add implementation of any methods of your controller is needed

private:
  rclcpp::WaitSet ref_subscriber_wait_set_;
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class DummyClassNameFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<CtrlType>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerReferenceMsg>(
      "/test_dummy_controller/reference", rclcpp::SystemDefaultsQoS());

    service_caller_node_ = std::make_shared<rclcpp::Node>("service_caller");
    slow_control_service_client_ = service_caller_node_->create_client<ControllerModeSrvType>(
      "/test_dummy_controller/set_slow_control_mode");
  }

  static void TearDownTestCase() {}

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(const std::string controller_name = "test_dummy_controller")
  {
    ASSERT_EQ(controller_->init(controller_name), controller_interface::return_type::OK);

    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(joint_command_values_.size());
    command_ifs.reserve(joint_command_values_.size());

    for (size_t i = 0; i < joint_command_values_.size(); ++i)
    {
      command_itfs_.emplace_back(hardware_interface::CommandInterface(
        command_joint_names_[i], interface_name_, &joint_command_values_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }
    // TODO(anyone): Add other command interfaces, if any

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_itfs_.reserve(joint_state_values_.size());
    state_ifs.reserve(joint_state_values_.size());

    for (size_t i = 0; i < joint_state_values_.size(); ++i)
    {
      state_itfs_.emplace_back(hardware_interface::StateInterface(
        command_joint_names_[i], interface_name_, &joint_state_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }
    // TODO(anyone): Add other state interfaces, if any

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void subscribe_to_controller_status_execute_update_and_get_messages(ControllerStateMsg & msg)
  {
    // create a new subscriber
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subs_callback = [&](const ControllerStateMsg::SharedPtr) {};
    auto subscription = test_subscription_node.create_subscription<ControllerStateMsg>(
      "/test_dummy_controller/controller_state", 10, subs_callback);

    // call update to publish the test value
    ASSERT_EQ(
      controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);

    // call update to publish the test value
    // since update doesn't guarantee a published message, republish until received
    int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
    rclcpp::WaitSet wait_set;          // block used to wait on message
    wait_set.add_subscription(subscription);
    while (max_sub_check_loop_count--)
    {
      controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01));
      // check if message has been received
      if (wait_set.wait(std::chrono::milliseconds(2)).kind() == rclcpp::WaitResultKind::Ready)
      {
        break;
      }
    }
    ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                              "controller/broadcaster update loop";

    // take message from subscription
    rclcpp::MessageInfo msg_info;
    ASSERT_TRUE(subscription->take(msg, msg_info));
  }

  // TODO(anyone): add/remove arguments as it suites your command message type
  void publish_commands(
    const rclcpp::Time & stamp, const std::vector<double> & displacements = {0.45}, 
    const std::vector<std::string> & joint_names = {"joint1_test"},
    const std::vector<double> & velocities = {0.0}, const double duration = 1.25)
  {
    auto wait_for_topic = [&](const auto topic_name)
    {
      size_t wait_count = 0;
      while (command_publisher_node_->count_subscribers(topic_name) == 0)
      {
        if (wait_count >= 5)
        {
          auto error_msg =
              std::string("publishing to ") + topic_name + " but no node subscribes to it";
          throw std::runtime_error(error_msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++wait_count;
      }
    };

    wait_for_topic(command_publisher_->get_topic_name());

    ControllerReferenceMsg msg;
    msg.header.stamp = stamp;
    msg.joint_names = joint_names;
    msg.displacements = displacements;
    msg.velocities = velocities;
    msg.duration = duration;

    command_publisher_->publish(msg);
  }

  std::shared_ptr<ControllerModeSrvType::Response> call_service(
    const bool slow_control, rclcpp::Executor & executor)
  {
    auto request = std::make_shared<ControllerModeSrvType::Request>();
    request->data = slow_control;

    bool wait_for_service_ret =
      slow_control_service_client_->wait_for_service(std::chrono::milliseconds(500));
    EXPECT_TRUE(wait_for_service_ret);
    if (!wait_for_service_ret)
    {
      throw std::runtime_error("Services is not available!");
    }
    auto result = slow_control_service_client_->async_send_request(request);
    EXPECT_EQ(executor.spin_until_future_complete(result), rclcpp::FutureReturnCode::SUCCESS);

    return result.get();
  }

protected:
  // TODO(anyone): adjust the members as needed

  // Controller-related parameters
  std::vector<std::string> command_joint_names_ = {"joint1"};
  std::vector<std::string> state_joint_names_ = {"joint1state"};
  std::string interface_name_ = "acceleration";
  std::array<double, 1> joint_state_values_ = {1.1};
  std::array<double, 1> joint_command_values_ = {101.101};
  // set command statically
  static constexpr double TEST_DISPLACEMENT = 23.24;
  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  // Test related parameters
  std::unique_ptr<TestableDummyClassName> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerReferenceMsg>::SharedPtr command_publisher_;
  rclcpp::Node::SharedPtr service_caller_node_;
  rclcpp::Client<ControllerModeSrvType>::SharedPtr slow_control_service_client_;
};

#endif  // TEMPLATES__ROS2_CONTROL__CONTROLLER__TEST_DUMMY_CHAINABLE_CONTROLLER_HPP_
