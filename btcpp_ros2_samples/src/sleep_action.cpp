#include "sleep_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool SleepAction::setGoal(RosActionNode::Goal& goal)
{
  auto timeout = getInput<unsigned>("msec");
  goal.msec_timeout = timeout.value();
  return true;
}

NodeStatus SleepAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
              wr.result->done ? "true" : "false");

  return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus SleepAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

BT::NodeStatus SleepAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO_THROTTLE(logger(), *clock(), 1000, "Feedback cycle %d", feedback->cycle);
  return NodeStatus::RUNNING;
}

void SleepAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

// Plugin registration.
// The class SleepAction will self register with name  "SleepAction".
CreateRosNodePlugin(SleepAction, "SleepAction");
