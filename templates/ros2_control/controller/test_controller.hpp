$LICENSE$

#ifndef TEST_DUMMY_PACKAGE_NAME_HPP_
#define TEST_DUMMY_PACKAGE_NAME_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "dummy_package_namespace/dummy_file_name.hpp"
#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

// TODO(anyone): replace the state and command message types
using ControllerStateMsg = control_msgs::msg::JointControllerState;
using ControllerCommandMsg = control_msgs::msg::JointJog;

namespace
{
constexpr auto NODE_SUCCESS =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

rclcpp::WaitResultKind wait_for(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  const auto timeout = std::chrono::seconds(10);
  return wait_set.wait(timeout).kind();
}

}  // namespace

// subclassing and friending so we can access member variables
class TestableDummyClassName : public dummy_package_namespace::DummyClassName
{
  FRIEND_TEST(DummyClassNameTest, joint_names_parameter_not_set);
  FRIEND_TEST(DummyClassNameTest, interface_parameter_not_set);
  FRIEND_TEST(DummyClassNameTest, all_parameters_set_configure_success);

public:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = dummy_package_namespace::DummyClassName::on_configure(previous_state);
    // Only if on_configure is successful create subscription
    if (ret == CallbackReturn::SUCCESS) {
      command_subscriber_wait_set_.add_subscription(command_subscriber_);
    }
    return ret;
  }

  /**
   * @brief wait_for_command blocks until a new ControllerCommandMsg is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function.
   *
   * @return true if new ControllerCommandMsg msg was received, false if timeout.
   */
  bool wait_for_commands(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    bool success =
      command_subscriber_wait_set_.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;
    if (success) {
      executor.spin_some();
    }
    return success;
  }

private:
  rclcpp::WaitSet command_subscriber_wait_set_;
};

class DummyClassNameTest : public ::testing::Test
{
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<TestableDummyClassName>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerCommandMsg>(
      "/test_dummy_package_namespace/commands", rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(bool set_parameters = true)
  {
    const auto result = controller_->init("test_dummy_package_namespace");
    ASSERT_EQ(result, controller_interface::return_type::OK);

    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(joint_command_values_.size());
    command_ifs.reserve(joint_command_values_.size());

    for (auto i = 0u; i < joint_command_values_.size(); ++i) {
      command_itfs_.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], interface_name_, &joint_command_values_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }
    // TODO(anyone): Add other command interfaces, if any

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_itfs_.reserve(joint_state_values_.size());
    state_ifs.reserve(joint_state_values_.size());

    for (auto i = 0u; i < joint_state_values_.size(); ++i) {
      state_itfs_.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], interface_name_, &joint_state_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }
    // TODO(anyone): Add other state interfaces, if any

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));

    if (set_parameters) {
      controller_->get_node()->set_parameter({"joints", joint_names_});
      controller_->get_node()->set_parameter({"interface_name", interface_name_});
    }
  }

  void subscribe_and_get_messages(ControllerStateMsg & msg)
  {
    // create a new subscriber
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subs_callback = [&](const ControllerStateMsg::SharedPtr) {};
    auto subscription = test_subscription_node.create_subscription<ControllerStateMsg>(
      "/test_dummy_package_namespace/state", 10, subs_callback);

    // call update to publish the test value
    rclcpp::Time node_time = controller_->get_node()->now();
    rclcpp::Duration duration = rclcpp::Duration::from_nanoseconds(1000000);  // 1ms
    ASSERT_EQ(controller_->update(node_time, duration), controller_interface::return_type::OK);

    // wait for message to be passed
    ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

    // take message from subscription
    rclcpp::MessageInfo msg_info;
    ASSERT_TRUE(subscription->take(msg, msg_info));
  }

  void publish_commands()
  {
    auto wait_for_topic = [&](const auto topic_name) {
      size_t wait_count = 0;
      while (command_publisher_node_->count_subscribers(topic_name) == 0) {
        if (wait_count >= 5) {
          auto error_msg =
            std::string("publishing to ") + topic_name + " but no node subscribes to it";
          throw std::runtime_error(error_msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++wait_count;
      }
    };

    wait_for_topic(command_publisher_->get_topic_name());

    ControllerCommandMsg msg;
    msg.joint_names = joint_names_;
    msg.displacements = {0.45};
    msg.velocities = {0.0};
    msg.duration = 1.25;

    command_publisher_->publish(msg);
  }

protected:
  // TODO(anyone): adjust the members as needed

  // Controller-related parameters
  std::vector<std::string> joint_names_ = {"joint1"};
  std::string interface_name_ = "my_interface";
  std::array<double, 1> joint_state_values_ = {1.1};
  std::array<double, 1> joint_command_values_ = {101.101};

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  // Test related parameters
  std::unique_ptr<TestableDummyClassName> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerCommandMsg>::SharedPtr command_publisher_;
};

// From the tutorial: https://www.sandordargo.com/blog/2019/04/24/parameterized-testing-with-gtest
class DummyClassNameTestParameterizedParameters
: public DummyClassNameTest,
  public ::testing::WithParamInterface<std::tuple<std::string, rclcpp::ParameterValue>>
{
public:
  virtual void SetUp() { DummyClassNameTest::SetUp(); }

  static void TearDownTestCase() { DummyClassNameTest::TearDownTestCase(); }

protected:
  void SetUpController(bool set_parameters = true)
  {
    DummyClassNameTest::SetUpController(set_parameters);
    controller_->get_node()->set_parameter({std::get<0>(GetParam()), std::get<1>(GetParam())});
  }
};

#endif  // TEST_DUMMY_PACKAGE_NAME_HPP_
