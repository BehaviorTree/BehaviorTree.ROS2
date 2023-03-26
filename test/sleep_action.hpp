#include <behaviortree_ros2/bt_action_node.hpp>
#include "behaviortree_ros2/action/sleep.hpp"

using namespace BT;

class SleepAction: public RosActionNode<behaviortree_ros2::action::Sleep>
{
public:
  SleepAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behaviortree_ros2::action::Sleep>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("msec")});
  }

  bool setGoal(Goal& goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
