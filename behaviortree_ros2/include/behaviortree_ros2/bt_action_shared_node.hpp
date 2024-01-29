// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2023 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_ros2/bt_action_error_code.hpp"

#include "behaviortree_ros2/ros_node_params.hpp"

namespace BT
{
/**
 * @brief Abstract class to wrap rclcpp_action::Client<>
 *
 * For instance, given the type AddTwoInts described in this tutorial:
 * https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
 *
 * the corresponding wrapper would be:
 *
 * class FibonacciNode: public RosActionSharedNode<action_tutorials_interfaces::action::Fibonacci>
 *
 * RosActionSharedNode will try to be non-blocking for the entire duration of the call.
 * The derived class must reimplement the virtual methods as described below.
 *
 * The name of the action will be determined as follows:
 *
 * 1. If a value is passes in the InputPort "action_name", use that
 * 2. Otherwise, use the value in RosNodeParams::default_port_value
 */
template<class ActionT, class Derived>
class RosActionSharedNode : public BT::ActionNodeBase
{

public:
  // Type definitions
  using ActionType = ActionT;
  using ActionClient = typename rclcpp_action::Client<ActionT>;
  using ActionClientPtr = std::shared_ptr<ActionClient>;
  using Goal = typename ActionT::Goal;
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
  using Feedback = typename ActionT::Feedback;

  /** To register this class into the factory, use:
   *
   *    factory.registerNodeType<>(node_name, params);
   *
   */
  explicit RosActionSharedNode(const std::string & instance_name,
                         const BT::NodeConfig& conf,
                         const RosNodeParams& params);

  virtual ~RosActionSharedNode();

  /**
   * @brief Any subclass of RosActionSharedNode that has ports must implement a
   * providedPorts method and call providedBasicPorts in it.
   *
   * @param addition Additional ports to add to BT port list
   * @return PortsList containing basic ports along with node-specific ports
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = {
      InputPort<std::string>("action_name", "__default__placeholder__", "Action server name")
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  /// @brief  Callback executed when the node is halted. Note that cancelGoal()
  /// is done automatically.
  virtual void onHalt() {}

  /** setGoal s a callback that allows the user to set
   *  the goal message (ActionT::Goal).
   *
   * @param goal  the goal to be sent to the action server.
   *
   * @return false if the request should not be sent. In that case,
   * RosActionSharedNode::onFailure(INVALID_GOAL) will be called.
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
  virtual BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> /*feedback*/)
  {
    return NodeStatus::RUNNING;
  }

  /** Callback invoked when something goes wrong.
   * It must return either SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFailure(ActionNodeErrorCode /*error*/)
  {
    return NodeStatus::FAILURE;
  }

  /// Method used to send a request to the Action server to cancel the current goal
  void cancelGoal();


  /// The default halt() implementation will call cancelGoal if necessary.
  void halt() override final;

  NodeStatus tick() override final;

protected:

  std::shared_ptr<rclcpp::Node> node_;
  std::string prev_action_name_;
  std::string action_name_;
  bool action_name_may_change_ = false;
  const std::chrono::milliseconds server_timeout_;
  const std::chrono::milliseconds wait_for_server_timeout_;
  static bool shared_resource_initialized;
  static ActionClientPtr action_client_;
  static std::mutex action_client_mutex_;


private:

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> callback_group_executor_;

  std::shared_future<typename GoalHandle::SharedPtr> future_goal_handle_;
  typename GoalHandle::SharedPtr goal_handle_;

  rclcpp::Time time_goal_sent_;
  NodeStatus on_feedback_state_change_;
  bool goal_received_;
  WrappedResult result_;

  bool createClient(const std::string &action_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template<class T, class D> inline
  RosActionSharedNode<T, D>::RosActionSharedNode(const std::string & instance_name,
                                  const NodeConfig &conf,
                                  const RosNodeParams &params):
  BT::ActionNodeBase(instance_name, conf),
  node_(params.nh),
  server_timeout_(params.server_timeout),
  wait_for_server_timeout_(params.wait_for_server_timeout)
{
  // Three cases:
  // - we use the default action_name in RosNodeParams when port is empty
  // - we use the action_name in the port and it is a static string.
  // - we use the action_name in the port and it is blackboard entry.

  // check port remapping
  auto portIt = config().input_ports.find("action_name");
  if(portIt != config().input_ports.end())
  {
    const std::string& bb_action_name = portIt->second;

    if(bb_action_name.empty() || bb_action_name == "__default__placeholder__")
    {
      if(params.default_port_value.empty()) {
        throw std::logic_error(
          "Both [action_name] in the InputPort and the RosNodeParams are empty.");
      }
      else {
        createClient(params.default_port_value);
      }
    }
    else if(!isBlackboardPointer(bb_action_name))
    {
      // If the content of the port "action_name" is not
      // a pointer to the blackboard, but a static string, we can
      // create the client in the constructor.
      createClient(bb_action_name);
    }
    else {
      action_name_may_change_ = true;
      // createClient will be invoked in the first tick().
    }
  }
  else {

    if(params.default_port_value.empty()) {
      throw std::logic_error(
        "Both [action_name] in the InputPort and the RosNodeParams are empty.");
    }
    else {
      createClient(params.default_port_value);
    }
  }
}

template<class ActionT, class Derived>
RosActionSharedNode<ActionT, Derived>::~RosActionSharedNode()
{
  RCLCPP_INFO(node_->get_logger(), "Destroying Action server for %s", action_name_.c_str());
  if (action_client_ && goal_handle_) {
    auto state = goal_handle_->get_status();
    if (state == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
      state == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
      try {
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
        if (callback_group_executor_) {
          callback_group_executor_->spin_some();
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception during goal cancellation in destructor: %s", e.what());
      }
    }
  }


  if (callback_group_executor_) {
    callback_group_executor_.reset();
  }

  if (action_client_) {
    action_client_.reset();
  }

  RCLCPP_INFO(node_->get_logger(), "Action server %s destroyed", action_name_.c_str());
}

template<class T, class D> inline
bool RosActionSharedNode<T, D>::createClient(const std::string& action_name)
{
  std::lock_guard<std::mutex> lock(action_client_mutex_);
  if (action_name.empty())
  {
      throw RuntimeError("action_name is empty");
  }

  this->action_name_ = action_name;

  if (!shared_resource_initialized)
  {
    RCLCPP_INFO(node_->get_logger(), "Creating action client for %s", action_name.c_str());
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    if (callback_group_executor_ == nullptr)
    {
        callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
    callback_group_executor_->add_callback_group(callback_group_, node_->get_node_base_interface());
    action_client_ = rclcpp_action::create_client<T>(node_, action_name, callback_group_);

    prev_action_name_ = action_name;
    shared_resource_initialized = true;
  }

  bool found = action_client_->wait_for_action_server(wait_for_server_timeout_);
  if(!found)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s: Action server with name '%s' is not reachable.",
                 name().c_str(), prev_action_name_.c_str());
  }
  return found;
}

template<class T, class D> inline
  NodeStatus RosActionSharedNode<T, D>::tick()
{
  // First, check if the action_client_ is valid and that the name of the
  // action_name in the port didn't change.
  // otherwise, create a new client
  if(!action_client_ || (status() == NodeStatus::IDLE && action_name_may_change_))
  {
    RCLCPP_INFO(node_->get_logger(), "Client not initialized or action_name changed");
    std::string action_name;
    getInput("action_name", action_name);
    if(prev_action_name_ != action_name)
    {
      createClient(action_name);
    }
  }

  //------------------------------------------
  auto CheckStatus = [](NodeStatus status)
  {
    if( !isStatusCompleted(status) )
    {
      throw std::logic_error("RosActionSharedNode: the callback must return either SUCCESS of FAILURE");
    }
    return status;
  };

  // first step to be done only at the beginning of the Action
  if (status() == BT::NodeStatus::IDLE)
  {
    RCLCPP_INFO(node_->get_logger(), "Client IDLE sending goal");
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
      on_feedback_state_change_ = onFeedback(feedback);
      if( on_feedback_state_change_ == NodeStatus::IDLE)
      {
        throw std::logic_error("onFeedback must not return IDLE");
      }
      emitWakeUpSignal();
    };
    //--------------------
    goal_options.result_callback =
      [this](const WrappedResult& result)
    {
      if (goal_handle_->get_goal_id() == result.goal_id) {
        RCLCPP_DEBUG( node_->get_logger(), "result_callback" );
        result_ = result;
        emitWakeUpSignal();
      }
    };
    //--------------------
    goal_options.goal_response_callback =
      [this](typename GoalHandle::SharedPtr const future_handle)
    {
      auto goal_handle_ = future_handle.get();
      if (!goal_handle_)
      {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_DEBUG(node_->get_logger(), "Goal accepted by server, waiting for result");
      }
    };
    //--------------------
    RCLCPP_INFO(node_->get_logger(), "Goal sent to server");
    future_goal_handle_ = action_client_->async_send_goal( goal, goal_options );
    time_goal_sent_ = node_->now();

    return NodeStatus::RUNNING;
  }

  if (status() == NodeStatus::RUNNING)
  {
    std::lock_guard<std::mutex> lock(action_client_mutex_);
    callback_group_executor_->spin_some();
    // FIRST case: check if the goal request has a timeout
    if( !goal_received_ )
    {
      auto nodelay = std::chrono::milliseconds(0);
      auto timeout = rclcpp::Duration::from_seconds( double(server_timeout_.count()) / 1000);

      auto ret = callback_group_executor_->spin_until_future_complete(future_goal_handle_, nodelay);

      if (ret != rclcpp::FutureReturnCode::SUCCESS)
      {
        if( (node_->now() - time_goal_sent_) > timeout )
        {
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
          return CheckStatus( onFailure( GOAL_REJECTED_BY_SERVER ) );
        }
      }
    }

    // SECOND case: onFeedback requested a stop
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

template<class T, class D> inline
  void RosActionSharedNode<T, D>::halt()
{
  if(status() == BT::NodeStatus::RUNNING)
  {
    cancelGoal();
    onHalt();
  }
}

template<class T, class D> inline
  void RosActionSharedNode<T, D>::cancelGoal()
{
  std::lock_guard<std::mutex> lock(action_client_mutex_);
  auto future_result = action_client_->async_get_result(goal_handle_);
  auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
  if (callback_group_executor_->spin_until_future_complete(future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR( node_->get_logger(), "Failed to cancel action server for [%s]",
                 prev_action_name_.c_str());
  }

  if (callback_group_executor_->spin_until_future_complete(future_result, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR( node_->get_logger(), "Failed to get result call failed :( for [%s]",
                 prev_action_name_.c_str());
  }
}

template <class ActionT, class Derived>
typename std::shared_ptr<rclcpp_action::Client<ActionT>> RosActionSharedNode<ActionT, Derived>::action_client_ = nullptr;

template <class ActionT, class Derived>
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> RosActionSharedNode<ActionT, Derived>::callback_group_executor_ = nullptr;

template <class ActionT, class Derived>
bool RosActionSharedNode<ActionT, Derived>::shared_resource_initialized = false;

template <class ActionT, class Derived>
std::mutex RosActionSharedNode<ActionT, Derived>::action_client_mutex_;

}  // namespace BT
