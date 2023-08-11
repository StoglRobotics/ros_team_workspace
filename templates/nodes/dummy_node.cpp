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

#include "dummy_package_namespace/dummy_node.hpp"

#include <memory>
#include <string>

#include "rclcpp/executors/single_threaded_executor.hpp"

namespace dummy_package_namespace
{
DummyNode::DummyNode(const std::string & node_name) : rclcpp::Node(node_name) {}

}  // namespace dummy_package_namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  const std::string node_name = "dummy_node";

  auto node = std::make_shared<dummy_package_namespace::DummyNode>(node_name);
  executor->add_node(node);

  RCLCPP_INFO(node->get_logger(), "Spinning node: '%s'.", node_name.c_str());

  executor->spin();
  rclcpp::shutdown();
  return 0;
}
