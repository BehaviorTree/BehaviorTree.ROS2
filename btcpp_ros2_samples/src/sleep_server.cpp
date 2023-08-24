#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "btcpp_ros2_interfaces/action/sleep.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

class SleepActionServer : public rclcpp::Node
{
public:
  using Sleep = btcpp_ros2_interfaces::action::Sleep;
  using GoalHandleSleep = rclcpp_action::ServerGoalHandle<Sleep>;

  explicit SleepActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("sleep_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Sleep>(
      this,
      "sleep_service",
      std::bind(&SleepActionServer::handle_goal, this, _1, _2),
      std::bind(&SleepActionServer::handle_cancel, this, _1),
      std::bind(&SleepActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Sleep>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Sleep::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with sleep time %d", goal->msec_timeout);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSleep> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSleep> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&SleepActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleSleep> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(5);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Sleep::Feedback>();
    auto result = std::make_shared<Sleep::Result>();

    rclcpp::Time deadline = get_clock()->now() + rclcpp::Duration::from_seconds( double(goal->msec_timeout) / 1000 );
    int cycle = 0;

    while( get_clock()->now() < deadline )
    {
      if (goal_handle->is_canceling())
      {
        result->done = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      feedback->cycle = cycle++;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->done = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class SleepActionServer


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SleepActionServer>();

  rclcpp::spin(node);

  return 0;
}
