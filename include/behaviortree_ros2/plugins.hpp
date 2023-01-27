#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"

#ifdef BT_PLUGIN_EXPORT

#if defined(_WIN32)
// MS-Windows NT
#define BTCPP_EXPORT extern "C" __declspec(dllexport)
#else
// Unix-like OSes
#define BTCPP_EXPORT extern "C" __attribute__ ((visibility ("default")))
#endif

#else
#define BTCPP_EXPORT static
#endif


// Use this macro to register a BT::RosActionNode/BT::RosServiceNode.
// - First argument: type to register (class derived from BT::RosActionNode/ BT::RosServiceNode)
// - Second argument: string with the registration name
// - Thirs argument: default name of the server; it might be overriden
//
// Usage example:
//   RegisterROSActionNode(SleepAction, "Sleep", "sleep_service");

#define RegisterROSActionNode(TYPE, REGISTRATION_NAME, DEFAULT_SERVER_NAME)     \
BTCPP_EXPORT void                                                               \
BT_RegisterROSBTActionFromPlugin(BT::BehaviorTreeFactory& factory,              \
                               rclcpp::Node::SharedPtr node,                    \
                               const std::string& server_name)                  \
{                                                                               \
  BT::NodeParams params;                                                        \
  params.server_name = server_name.empty() ? DEFAULT_SERVER_NAME : server_name; \
  params.nh = node;                                                             \
  factory.registerNodeType<TYPE>(REGISTRATION_NAME, params);                    \
}                                                                               \



inline
void RegisterRosActionNode(BT::BehaviorTreeFactory& factory,
                           const std::string& filepath,
                           rclcpp::Node::SharedPtr node,
                           const std::string& server_name = {})
{
  BT::SharedLibrary loader(filepath);
  typedef void (*Func)(BT::BehaviorTreeFactory&, rclcpp::Node::SharedPtr, const std::string&);
  auto func = (Func)loader.getSymbol("BT_RegisterROSBTActionFromPlugin");
  func(factory, node, server_name);
}
