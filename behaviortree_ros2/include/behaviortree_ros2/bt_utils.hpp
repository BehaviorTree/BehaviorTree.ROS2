// Copyright 2024 Marq Rasmussen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright
// notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <functional>
#include <memory>
#include <thread>

#include "btcpp_ros2_interfaces/msg/node_status.hpp"

#include "behaviortree_cpp/bt_factory.h"

// auto-generated header, created by generate_parameter_library
#include "behaviortree_ros2/bt_executor_parameters.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace BT
{
/**
 * @brief Convert BT::NodeStatus into Action Server feedback message NodeStatus
 *
 * @param status Current status of the executing BehaviorTree
 * @return NodeStatus used to publish feedback to the Action Client
 */
btcpp_ros2_interfaces::msg::NodeStatus ConvertNodeStatus(BT::NodeStatus& status);

/**
 * @brief Function the uses ament_index_cpp to get the package path of the parameter specified by the user
 *
 * @param parameter_value String containing 'package_name/subfolder' for the directory path to look up
 * @return Full path to the directory specified by the parameter_value
 */
std::string GetDirectoryPath(const std::string& parameter_value);

/**
 * @brief Function to load BehaviorTree xml files from a specific directory
 *
 * @param factory BehaviorTreeFactory to register the BehaviorTrees into
 * @param directory_path Full path to the directory to search for BehaviorTree definitions
 */
void LoadBehaviorTrees(BT::BehaviorTreeFactory& factory,
                       const std::string& directory_path);

/**
 * @brief Function to load a BehaviorTree ROS plugin (or standard BT.CPP plugins)
 *
 * @param factory BehaviorTreeFactory to register the plugin into
 * @param file_path Full path to the directory to search for BehaviorTree plugin
 * @param params parameters passed to the ROS plugin
 */
void LoadPlugin(BT::BehaviorTreeFactory& factory, const std::filesystem::path& file_path,
                BT::RosNodeParams params);

/** @brief Function to load all plugins from the specified package
 *
 * @param params ROS parameters that contain the package name to load plugins from
 * @param factory BehaviorTreeFactory to register the plugins into
 * @param node node pointer that is shared with the ROS based Behavior plugins
 */
void RegisterPlugins(bt_server::Params& params, BT::BehaviorTreeFactory& factory,
                     rclcpp::Node::SharedPtr node);

/**
 * @brief Function to register all Behaviors and BehaviorTrees from user specified packages
 *
 * @param params ROS parameters that contain lists of packages to load
 * plugins, ros_plugins and BehaviorTrees from
 * @param factory BehaviorTreeFactory to register into
 * @param node node pointer that is shared with the ROS based Behavior plugins
 */
void RegisterBehaviorTrees(bt_server::Params& params, BT::BehaviorTreeFactory& factory,
                           rclcpp::Node::SharedPtr node);

}  // namespace BT
