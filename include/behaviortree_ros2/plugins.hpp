#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/bt_action_node.hpp"

// Use this macro to register a BT::RosActionNode.
// - First argument: type to register (class derived from BT::RosActionNode)
// - Second argument: string with the registration name
//
// Usage example:
//   RegisterROSActionNode(SleepAction, "Sleep");

#define RegisterROSActionNode(TYPE, REGISTRATION_NAME)              \
BTCPP_EXPORT void                                                   \
BT_RegisterROSActionFromPlugin(BT::BehaviorTreeFactory& factory,    \
                               const BT::ActionNodeParams& params)  \
{                                                                   \
  factory.registerNodeType<TYPE>(REGISTRATION_NAME, params);        \
}                                                                   \



inline
void RegisterRosActionNode(BT::BehaviorTreeFactory& factory,
                           const std::string& filepath,
                           const BT::ActionNodeParams& params)
{
  BT::SharedLibrary loader(filepath);
  typedef void (*Func)(BT::BehaviorTreeFactory&,
                       const BT::ActionNodeParams&);
  auto func = (Func)loader.getSymbol("BT_RegisterROSActionFromPlugin");
  func(factory, params);
}




