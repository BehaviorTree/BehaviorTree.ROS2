#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/node_params.hpp"

<<<<<<< HEAD
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
=======
// Use this macro to register a BT::RosActionNode.
// - First argument: type to register (class derived from BT::RosActionNode)
// - Second argument: string with the registration name
>>>>>>> parent/v4
//
// Usage example:
//   RegisterROSActionNode(SleepAction, "Sleep");

<<<<<<< HEAD
#define RegisterROSBTNode(TYPE, REGISTRATION_NAME, DEFAULT_SERVER_NAME)         \
BTCPP_EXPORT void                                                               \
BT_RegisterROSBTNodeFromPlugin(BT::BehaviorTreeFactory& factory,                \
                               rclcpp::Node::SharedPtr node,                    \
                               const std::string& server_name)                  \
{                                                                               \
  BT::NodeParams params;                                                        \
  params.default_server_name =                                                  \
      server_name.empty() ? DEFAULT_SERVER_NAME : server_name;                  \
  params.nh = node;                                                             \
  factory.registerNodeType<TYPE>(REGISTRATION_NAME, params);                    \
}                                                                               \
=======
#define RegisterROSActionNode(TYPE, REGISTRATION_NAME)              \
BTCPP_EXPORT void                                                   \
BT_RegisterROSActionFromPlugin(BT::BehaviorTreeFactory& factory,    \
                               const BT::ActionNodeParams& params)  \
{                                                                   \
  factory.registerNodeType<TYPE>(REGISTRATION_NAME, params);        \
}                                                                   \
>>>>>>> parent/v4



inline
void RegisterRosBTNode(BT::BehaviorTreeFactory& factory,
                           const std::string& filepath,
<<<<<<< HEAD
                           rclcpp::Node::SharedPtr node,
                           const std::string& server_name = {})
{
  BT::SharedLibrary loader(filepath);
  typedef void (*Func)(BT::BehaviorTreeFactory&, rclcpp::Node::SharedPtr, const std::string&);
  auto func = (Func)loader.getSymbol("BT_RegisterROSBTNodeFromPlugin");
  func(factory, node, server_name);
=======
                           const BT::ActionNodeParams& params)
{
  BT::SharedLibrary loader(filepath);
  typedef void (*Func)(BT::BehaviorTreeFactory&,
                       const BT::ActionNodeParams&);
  auto func = (Func)loader.getSymbol("BT_RegisterROSActionFromPlugin");
  func(factory, params);
>>>>>>> parent/v4
}
