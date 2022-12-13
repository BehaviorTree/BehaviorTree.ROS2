// Copyright (c) 2018 Intel Corporation
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

#ifndef BEHAVIORTREE_ROS2__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_
#define BEHAVIORTREE_ROS2__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace BT
{

/**
 * @brief A DecoratorNode that ticks its child at a specified rate
 */
class RateController : public DecoratorNode
{
public:
  /**
   * @brief A constructor for RateController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RateController(
    const std::string & name,
    const NodeConfig & conf);

  /**
   * @brief Creates list of BT ports
   * @return PortsList Containing node-specific ports
   */
  static PortsList providedPorts()
  {
    return {
      InputPort<double>("hz", 10.0, "Rate")
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return NodeStatus Status of tick execution
   */
  NodeStatus tick() override;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period_;
  bool first_time_;
};

}  // namespace BT

#endif  // BEHAVIORTREE_ROS2__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_
