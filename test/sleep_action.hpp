#include <behaviortree_ros2/bt_action_node.hpp>
#include "behaviortree_ros2/node_params.hpp"
#include "behaviortree_ros2/action/sleep.hpp"

using namespace BT;

class SleepAction: public RosActionNode<behaviortree_ros2::action::Sleep>
{
public:
  SleepAction(const std::string& name,
              const BT::NodeConfig& conf,
              const NodeParams& params,
              typename std::shared_ptr<ActionClient> action_client = {})
    : RosActionNode<behaviortree_ros2::action::Sleep>(name, conf, params, action_client)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("msec")});
  }

  bool setGoal(Goal& goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
