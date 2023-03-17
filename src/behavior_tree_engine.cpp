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


#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/behavior_tree_engine.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/node_params.hpp"

namespace BT
{

BehaviorTreeEngine::BehaviorTreeEngine(const std::string& name, const std::vector<std::string> & plugin_libraries, const std::vector<std::string> & default_server_names) 
  : node_(rclcpp::Node::make_shared(name, rclcpp::NodeOptions().arguments({
    "--ros-args", "-r", name + ":" + std::string("__node:=") + name
  })))
{
  SharedLibrary loader;
  NodeParams params;
  params.nh = node_;
  bool use_default_server_names = default_server_names.size() == plugin_libraries.size();
  if (!use_default_server_names) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BehaviorTreeEngine"),
      "Number of default server names does not match number of plugin libraries. Ignoring values...");
  }
  for (size_t i = 0; i < plugin_libraries.size(); i++) {
    if (use_default_server_names) {
      params.default_server_name = default_server_names[i];
    }
    RegisterRosBTNode(factory_, loader.getOSName(plugin_libraries[i]), params);
  }
  std::string tree_model = writeTreeNodesModelXML(factory_);
  std::ofstream xml_file;
  xml_file.open("/tree_model.xml");
  xml_file << tree_model;
  xml_file.close();
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

void
BehaviorTreeEngine::registerTreesFromDirectory(const std::string & search_directory)
{
  using std::filesystem::directory_iterator;
  for (auto const& entry : directory_iterator(search_directory)) 
  {
    if( entry.path().extension() == ".xml")
    {
      factory_.registerBehaviorTreeFromFile(entry.path().string());
    }
  }
}

void
BehaviorTreeEngine::registerTreeFromFile(const std::string & file_path)
{
  factory_.registerBehaviorTreeFromFile(file_path);
}

void
BehaviorTreeEngine::registerTreeFromText(const std::string & xml_string)
{
  factory_.registerBehaviorTreeFromText(xml_string);
}

Tree
BehaviorTreeEngine::createTree(const std::string & tree_id, Blackboard::Ptr blackboard)
{
  return factory_.createTree(tree_id, blackboard);
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
