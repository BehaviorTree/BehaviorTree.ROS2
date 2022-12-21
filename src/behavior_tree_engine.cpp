// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
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


#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/behavior_tree_engine.hpp"

namespace BT
{

BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries)
{
  SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

BtStatus
BehaviorTreeEngine::run(
  Tree * tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  NodeStatus result = NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes
  try {
    while (rclcpp::ok() && result == NodeStatus::RUNNING) {
      if (cancelRequested()) {
        tree->rootNode()->haltNode();
        return BtStatus::CANCELED;
      }

      result = tree->tickOnce();

      onLoop();

      loopRate.sleep();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BehaviorTreeEngine"),
      "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
    return BtStatus::FAILED;
  }

  return (result == NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

Tree
BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string,
  Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(xml_string, blackboard);
}

Tree
BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path,
  Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromFile(file_path, blackboard);
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
void
BehaviorTreeEngine::haltAllActions(TreeNode * root_node)
{
  if (!root_node) {
    return;
  }

  // this halt signal should propagate through the entire tree.
  root_node->haltNode();

  // but, just in case...
  auto visitor = [](TreeNode * node) {
      if (node->status() == NodeStatus::RUNNING) {
        node->haltNode();
      }
    };
  applyRecursiveVisitor(root_node, visitor);
}

}  // namespace BT