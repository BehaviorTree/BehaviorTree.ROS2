#include "sleep_action_shared.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool SleepAction::setGoal(RosActionSharedNode::Goal &goal)
{
  RCLCPP_INFO(rclcpp::get_logger("test"), "%s, set goal", name().c_str());
  auto timeout = getInput<unsigned>("msec");
  goal.msec_timeout = timeout.value();
  return true;
}

NodeStatus SleepAction::onResultReceived(const RosActionSharedNode::WrappedResult &wr)
{
  RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(),
               wr.result->done ? "true" : "false" );
  RCLCPP_INFO(rclcpp::get_logger("test"), " %s action pointer: %p", name().c_str(), reinterpret_cast<void*>(action_client_.get()));

  return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus SleepAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  RCLCPP_INFO(rclcpp::get_logger("test"), " %s action pointer: %p", name().c_str(), reinterpret_cast<void*>(action_client_.get()));
  return NodeStatus::FAILURE;
}

void SleepAction::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  RCLCPP_INFO(rclcpp::get_logger("test"), " %s action pointer: %p", name().c_str(), reinterpret_cast<void*>(action_client_.get()));
}

// Plugin registration.
// The class SleepAction will self register with name  "Sleep".
CreateRosNodePlugin(SleepAction, "Sleep");
