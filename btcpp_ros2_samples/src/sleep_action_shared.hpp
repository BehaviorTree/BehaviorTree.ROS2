#include "behaviortree_ros2/bt_action_shared_node.hpp"
#include "btcpp_ros2_interfaces/action/sleep.hpp"

using namespace BT;

class SleepAction: public RosActionSharedNode<btcpp_ros2_interfaces::action::Sleep, SleepAction>
{
public:
  SleepAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionSharedNode<btcpp_ros2_interfaces::action::Sleep, SleepAction>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("msec")});
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
