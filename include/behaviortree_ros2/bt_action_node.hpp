#ifndef BEHAVIOR_TREE_ROS2__BT_ACTION_NODE_HPP_
#define BEHAVIOR_TREE_ROS2__BT_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp_action/rclcpp_action.hpp"

namespace BT
{

struct ActionNodeParams
{
  std::shared_ptr<rclcpp::Node> nh;
  std::string action_name;
  std::chrono::milliseconds server_timeout = std::chrono::milliseconds(1000);
};

enum ActionNodeErrorCode
{
  SERVER_UNREACHABLE,
  SEND_GOAL_TIMEOUT,
  GOAL_REJECTED_BY_SERVER,
  ACTION_ABORTED,
  ACTION_CANCELLED,
  INVALID_GOAL
};

/**
 * @brief Abstract class representing use to call a ROS2 Action (client).
 *
 * It will try to be non-blocking for the entire duration of the call.
 * The derived class whould reimplement the virtual methods as described below.
 */
template<class ActionT>
class RosActionNode : public BT::ActionNodeBase
{

public:
  // Type definitions
  using ActionType = ActionT;
  using ActionClient = typename rclcpp_action::Client<ActionT>;
  using Goal = typename ActionT::Goal;
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
  using Feedback = typename ActionT::Feedback;
  using Params = ActionNodeParams;

  /** You are not supposed to instantiate this class directly, the factory will do it.
   * To register this class into the factory, use:
   *
   *    RegisterRosAction<DerivedClasss>(factory, params)
   *
   * Note that if the external_action_client is not set, the constructor will build its own.
   * */
  explicit RosActionNode(const std::string & instance_name,
                         const BT::NodeConfig& conf,
                         const ActionNodeParams& params,
                         typename std::shared_ptr<ActionClient> external_action_client = {});

  virtual ~RosActionNode() = default;

  NodeStatus tick() override final;

  /// The default halt() implementation will call cancelGoal is necessary.
  void halt() override;

  /** setGoal is a callback invoked to return the goal message (ActionT::Goal).
   * If conditions are not met, it should return "false" and the BT::Action
   * will return FAILURE.
   */
  virtual bool setGoal(Goal& goal) = 0;

  /** Callback invoked when the result is received by the server.
   * It is up to the user to define if the action returns SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onResultReceived(const WrappedResult& result) = 0;

  /** Callback invoked when the feedback is received.
   * It generally returns RUNNING, but the user can also use this callback to cancel the
   * current action and return SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFeeback(const std::shared_ptr<const Feedback> feedback)
  {
    return NodeStatus::RUNNING;
  }

  /** Callback invoked when something goes wrong.
   * It must return either SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error)
  {
    return NodeStatus::FAILURE;
  }

  /// Method used to send a request to the Action server to cancel the current goal
  void cancelGoal();

protected:

  std::shared_ptr<rclcpp::Node> node_;
  const std::string action_name_;
  const std::chrono::milliseconds server_timeout_;

private:

  typename std::shared_ptr<ActionClient> action_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::shared_future<typename GoalHandle::SharedPtr> future_goal_handle_;
  typename GoalHandle::SharedPtr goal_handle_;

  rclcpp::Time time_goal_sent_;
  NodeStatus on_feedback_state_change_;
  bool goal_received_;
  WrappedResult result_;
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template<class T> inline
  RosActionNode<T>::RosActionNode(const std::string & instance_name,
                                  const NodeConfig &conf,
                                  const ActionNodeParams& params,
                                  typename std::shared_ptr<ActionClient> external_action_client):
  BT::ActionNodeBase(instance_name, conf),
  node_(params.nh),
  action_name_(params.action_name),
  server_timeout_(params.server_timeout)
{
  if( external_action_client )
  {
    action_client_ = external_action_client;
  }
  else
  {
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    action_client_ = rclcpp_action::create_client<T>(node_, action_name_, callback_group_);

    bool found = action_client_->wait_for_action_server(server_timeout_);
    if(!found)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Action server [%s] is not reachable. This will be checked only once", action_name_.c_str());
    }
  }
}

template<class T> inline
  NodeStatus RosActionNode<T>::tick()
{
  auto CheckStatus = [](NodeStatus status)
  {
    if( status != NodeStatus::SUCCESS && status != NodeStatus::FAILURE )
    {
      throw std::logic_error("RosActionNode: the callback must return either SUCCESS of FAILURE");
    }
    return status;
  };

  // first step to be done only at the beginning of the Action
  if (status() == BT::NodeStatus::IDLE)
  {
    setStatus(NodeStatus::RUNNING);

    goal_received_ = false;
    future_goal_handle_ = {};
    on_feedback_state_change_ = NodeStatus::RUNNING;
    result_ = {};

    Goal goal;

    if( !setGoal(goal) )
    {
      return CheckStatus( onFailure(INVALID_GOAL) );
    }

    typename ActionClient::SendGoalOptions goal_options;

    //--------------------
    goal_options.feedback_callback =
      [this](typename GoalHandle::SharedPtr,
             const std::shared_ptr<const Feedback> feedback)
    {
      on_feedback_state_change_ = onFeeback(feedback);
      if( on_feedback_state_change_ == NodeStatus::IDLE)
      {
        throw std::logic_error("onFeeback must not retunr IDLE");
      }
      emitWakeUpSignal();
    };
    //--------------------
    goal_options.result_callback =
      [this](const WrappedResult& result)
    {
      RCLCPP_INFO( node_->get_logger(), "result_callback" );
      result_ = result;
      emitWakeUpSignal();
    };
    //--------------------
    goal_options.goal_response_callback =
      [this](std::shared_future<typename GoalHandle::SharedPtr> const future_handle)
    {
      auto goal_handle_ = future_handle.get();
      if (!goal_handle_)
      {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
      }
    };
    //--------------------

    future_goal_handle_ = action_client_->async_send_goal( goal, goal_options );
    time_goal_sent_ = node_->now();

    return NodeStatus::RUNNING;
  }

  if (status() == NodeStatus::RUNNING)
  {
    rclcpp::spin_some(node_);

    // FIRST case: check if the goal request has a timeout
    if( !goal_received_ )
    {
      auto nodelay = std::chrono::milliseconds(0);
      auto timeout = rclcpp::Duration::from_seconds( double(server_timeout_.count()) / 1000);

      if (rclcpp::spin_until_future_complete(node_, future_goal_handle_, nodelay) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_WARN( node_->get_logger(), "waiting goal confirmation" );
        if( (node_->now() - time_goal_sent_) > timeout )
        {
          RCLCPP_WARN( node_->get_logger(), "TIMEOUT" );
          return CheckStatus( onFailure(SEND_GOAL_TIMEOUT) );
        }
        else{
          return NodeStatus::RUNNING;
        }
      }
      else
      {
        goal_received_ = true;
        goal_handle_ = future_goal_handle_.get();
        future_goal_handle_ = {};

        if (!goal_handle_) {
          throw std::runtime_error("Goal was rejected by the action server");
        }
      }
    }

    // SECOND case: onFeeback requested a stop
    if( on_feedback_state_change_ != NodeStatus::RUNNING )
    {
      cancelGoal();
      return on_feedback_state_change_;
    }
    // THIRD case: result received, requested a stop
    if( result_.code != rclcpp_action::ResultCode::UNKNOWN)
    {
      if( result_.code == rclcpp_action::ResultCode::ABORTED )
      {
        return CheckStatus( onFailure( ACTION_ABORTED ) );
      }
      else if( result_.code == rclcpp_action::ResultCode::CANCELED )
      {
        return CheckStatus( onFailure( ACTION_CANCELLED ) );
      }
      else{
        return CheckStatus( onResultReceived( result_ ) );
      }
    }
  }
  return NodeStatus::RUNNING;
}

template<class T> inline
  void RosActionNode<T>::halt()
{
  if( status() == NodeStatus::RUNNING )
  {
    cancelGoal();
  }
}

template<class T> inline
  void RosActionNode<T>::cancelGoal()
{
  auto future_cancel = action_client_->async_cancel_goal(goal_handle_);

  if (rclcpp::spin_until_future_complete(node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR( node_->get_logger(),
                 "Failed to cancel action server for %s", action_name_.c_str());
  }
}




}  // namespace BT

#endif  // BEHAVIOR_TREE_ROS2__BT_ACTION_NODE_HPP_
