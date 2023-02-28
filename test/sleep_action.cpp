#include "sleep_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool SleepAction::setGoal(RosActionNode::Goal &goal)
{
  auto timeout = getInput<unsigned>("msec");
  goal.msec_timeout = timeout.value();
  return true;
}

NodeStatus SleepAction::onResultReceived(const RosActionNode::WrappedResult &wr)
{
  RCLCPP_INFO( node_->get_logger(), "onResultReceived %d", wr.result->done );
  return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus SleepAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "onFailure %d", error );
  return NodeStatus::FAILURE;
}

// Plugin registration.
// The class SleepAction will self register with name  "Sleep"
// and, by default, it will connect to the "sleep_service".
RegisterROSActionNode(SleepAction, "Sleep", "sleep_service");
