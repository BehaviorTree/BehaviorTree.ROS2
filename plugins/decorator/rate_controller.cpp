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

#include <chrono>
#include <string>

#include "behaviortree_ros2/plugins/decorator/rate_controller.hpp"

namespace BT
{

RateController::RateController(
  const std::string & name,
  const NodeConfig & conf)
: DecoratorNode(name, conf),
  first_time_(false)
{
  double hz = 1.0;
  getInput("hz", hz);
  period_ = 1.0 / hz;
}

NodeStatus RateController::tick()
{
  if (status() == NodeStatus::IDLE) {
    // Reset the starting point since we're starting a new iteration of
    // the rate controller (moving from IDLE to RUNNING)
    start_ = std::chrono::high_resolution_clock::now();
    first_time_ = true;
  }

  setStatus(NodeStatus::RUNNING);

  // Determine how long its been since we've started this iteration
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  // Now, get that in seconds
  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  // The child gets ticked the first time through and any time the period has
  // expired. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  if (first_time_ || (child_node_->status() == NodeStatus::RUNNING) ||
    seconds.count() >= period_)
  {
    first_time_ = false;
    const NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case NodeStatus::RUNNING:
        return NodeStatus::RUNNING;

      case NodeStatus::SUCCESS:
        start_ = std::chrono::high_resolution_clock::now();  // Reset the timer
        return NodeStatus::SUCCESS;

      case NodeStatus::FAILURE:
      default:
        return NodeStatus::FAILURE;
    }
  }

  return status();
}

}  // namespace BT

#include <behaviortree_cpp/bt_factory.h>
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BT::RateController>("RateController");
}
