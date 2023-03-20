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

#ifndef BEHAVIOR_TREE_ROS2__BEHAVIOR_TREE_ENGINE_HPP_
#define BEHAVIOR_TREE_ROS2__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"


namespace BT
{

/**
 * @enum nav2_behavior_tree::BtStatus
 * @brief An enum class representing BT execution status
 */
enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

/**
 * @class nav2_behavior_tree::BehaviorTreeEngine
 * @brief A class to create and handle behavior trees
 */
class BehaviorTreeEngine
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::BehaviorTreeEngine
   * @param plugin_libraries vector of BT plugin library names to load
   */
  explicit BehaviorTreeEngine(const std::string& name, const std::vector<std::string> & plugin_libraries, const std::vector<std::string> & default_server_names);
  virtual ~BehaviorTreeEngine() {}

  /**
   * @brief Function to execute a BT at a specific rate
   * @param tree BT to execute
   * @param onLoop Function to execute on each iteration of BT execution
   * @param cancelRequested Function to check if cancel was requested during BT execution
   * @param loopTimeout Time period for each iteration of BT execution
   * @return nav2_behavior_tree::BtStatus Status of BT execution
   */
  BtStatus run(
    Tree * tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  void registerTreesFromDirectory(const std::string & search_directory);

  void registerTreeFromFile(const std::string & file_path);

  void registerTreeFromText(const std::string & xml_string);

  template<typename T>
  void registerEnums()
  {
    factory_.registerScriptingEnums<T>();
  }

  Tree createTree(const std::string & tree_id, Blackboard::Ptr blackboard);

  /**
   * @brief Function to create a BT from a XML string
   * @param xml_string XML string representing BT
   * @param blackboard Blackboard for BT
   * @return Tree Created behavior tree
   */
  Tree createTreeFromText(
    const std::string & xml_string,
    Blackboard::Ptr blackboard);

  /**
   * @brief Function to create a BT from an XML file
   * @param file_path Path to BT XML file
   * @param blackboard Blackboard for BT
   * @return Tree Created behavior tree
   */
  Tree createTreeFromFile(
    const std::string & file_path,
    Blackboard::Ptr blackboard);

  /**
   * @brief Function to explicitly reset all BT nodes to initial state
   * @param root_node Pointer to BT root node
   */
  void haltAllActions(TreeNode * root_node);

protected:
  // The factory that will be used to dynamically construct the behavior tree
  BehaviorTreeFactory factory_;
  std::shared_ptr<rclcpp::Node> node_;
};

}  // namespace BT

#endif  // BEHAVIOR_TREE_ROS2__BEHAVIOR_TREE_ENGINE_HPP_
