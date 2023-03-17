#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/node_params.hpp"

// Use this macro to register a BT::RosActionNode/BT::RosServiceNode/BT::RosTopicNode/BT::RosTopicPubNode.
// - First argument: type to register (class derived from one of the above)
// - Second argument: string with the registration name
//
// Usage example:
//   RegisterROSBTNode(SleepAction, "Sleep");

#define RegisterROSBTNode(TYPE, REGISTRATION_NAME)                  \
BTCPP_EXPORT void                                                   \
BT_RegisterROSBTFromPlugin(BT::BehaviorTreeFactory& factory,        \
                               const BT::NodeParams& params)        \
{                                                                   \
  factory.registerNodeType<TYPE>(REGISTRATION_NAME, params);        \
}                                                                   \



inline
void RegisterRosBTNode(BT::BehaviorTreeFactory& factory,
                           const std::string& filepath,
                           const BT::NodeParams& params)
{
  BT::SharedLibrary loader(filepath);
  typedef void (*Func)(BT::BehaviorTreeFactory&,
                       const BT::NodeParams&);
  auto func = (Func)loader.getSymbol("BT_RegisterROSBTFromPlugin");
  func(factory, params);
}
