#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/bt_action_node.hpp"

#ifdef BT_PLUGIN_EXPORT

#if defined(_WIN32)
// MS-Windows NT
#define BTCPP_EXPORT extern "C" __declspec(dllexport)
#else
// Unix-like OSes
#define BTCPP_EXPORT extern "C" __attribute__ ((visibility ("default")))
#endif

#else
#define BTCPP_EXPORT
#endif


// Use this macro to register a BT::RosActionNode.
// - First argument: type to register (class derived from BT::RosActionNode)
// - Second argument: string with the registration name
// - Thirs argument: default name of the action; it might be overriden
//
// Usage example:
//   RegisterROSActionNode(SleepAction, "Sleep", "sleep_service");

#define RegisterROSActionNode(TYPE, REGISTRATION_NAME, DEFAULT_ACTION_NAME)     \
BTCPP_EXPORT void                                                               \
BT_RegisterROSActionFromPlugin(BT::BehaviorTreeFactory& factory,                \
                               rclcpp::Node::SharedPtr node,                    \
                               const std::string& action_name)                  \
{                                                                               \
  BT::ActionNodeParams params;                                                  \
  params.action_name = action_name.empty() ? DEFAULT_ACTION_NAME : action_name; \
  params.nh = node;                                                             \
  factory.registerNodeType<TYPE>(REGISTRATION_NAME, params);                    \
}                                                                               \



inline
void RegisterRosActionNode(BT::BehaviorTreeFactory& factory,
                           const std::string& filepath,
                           rclcpp::Node::SharedPtr node,
                           const std::string& action_name = {})
{
  BT::SharedLibrary loader(filepath);
  typedef void (*Func)(BT::BehaviorTreeFactory&, rclcpp::Node::SharedPtr, const std::string&);
  auto func = (Func)loader.getSymbol("BT_RegisterROSActionFromPlugin");
  func(factory, node, action_name);
}




