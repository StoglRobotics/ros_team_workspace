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

#ifndef TEMPLATES__NODES__DUMMY_PACKAGE_NAMESPACE__DUMMY_NODE_HPP_
#define TEMPLATES__NODES__DUMMY_PACKAGE_NAMESPACE__DUMMY_NODE_HPP_

#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dummy_package_namespace
{
class DummyNode : public rclcpp::Node
{
public:
  explicit DummyNode(const std::string & node_name = "dummy_node");

  virtual ~DummyNode() = default;
};

}  // namespace dummy_package_namespace

#endif  // TEMPLATES__NODES__DUMMY_PACKAGE_NAMESPACE__DUMMY_NODE_HPP_
