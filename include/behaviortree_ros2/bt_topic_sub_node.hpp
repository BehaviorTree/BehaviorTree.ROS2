// Copyright (c) 2023 Davide Faconti, Unmanned Life
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
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "behaviortree_ros2/ros_node_params.hpp"

namespace BT
{

/**
 * @brief Abstract class to wrap a Topic subscriber.
 * Considering the example in the tutorial:
 * https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
 *
 * The corresponding wrapper would be:
 *
 * class SubscriberNode: RosTopicSubNode<std_msgs::msg::String>
 *
 * The name of the topic will be determined as follows:
 *
 * 1. If a value is passes in the InputPort "topic_name", use that
 * 2. Otherwise, use the value in RosNodeParams::default_port_value
 */
template<class TopicT>
class RosTopicSubNode : public BT::ConditionNode
{

public:
  // Type definitions
  using Subscriber = typename rclcpp::Subscription<TopicT>;

  /** You are not supposed to instantiate this class directly, the factory will do it.
   * To register this class into the factory, use:
   *
   *    RegisterRosAction<DerivedClasss>(factory, params)
   *
   * Note that if the external_action_client is not set, the constructor will build its own.
   * */
  explicit RosTopicSubNode(const std::string & instance_name,
                           const BT::NodeConfig& conf,
                           const RosNodeParams& params);

  virtual ~RosTopicSubNode() = default;

  /**
   * @brief Any subclass of RosTopicNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = {
      InputPort<std::string>("topic_name", "__default__placeholder__", "Topic name")
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

  NodeStatus tick() override final;

  /** topicCallback is the callback invoked when a message is received.
   */
  virtual void topicCallback(const std::shared_ptr<TopicT> msg);

  /** Callback invoked in the tick. You must return either SUCCESS of FAILURE
   *
   * @param last_msg the latest message received since the last tick.
   * it might be empty.
   * @return the new status of the Node, based on last_msg
   */
  virtual BT::NodeStatus onTick(const typename TopicT::SharedPtr& last_msg) = 0;

protected:

  std::shared_ptr<rclcpp::Node> node_;
  std::string prev_topic_name_;
  bool topic_name_may_change_ = false;

private:

  std::shared_ptr<Subscriber> subscriber_;
  typename TopicT::SharedPtr last_msg_;
  bool is_message_received_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  bool createSubscriber(const std::string& topic_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template<class T> inline
  RosTopicSubNode<T>::RosTopicSubNode(const std::string & instance_name,
                                      const NodeConfig &conf,
                                      const RosNodeParams& params)
    : BT::ConditionNode(instance_name, conf),
      node_(params.nh)
{ 
  // check port remapping
  auto portIt = config().input_ports.find("topic_name");
  if(portIt != config().input_ports.end())
  {
    const std::string& bb_topic_name = portIt->second;

    if(bb_topic_name.empty() || bb_topic_name == "__default__placeholder__")
    {
      if(params.default_port_value.empty()) {
        throw std::logic_error(
          "Both [topic_name] in the InputPort and the RosNodeParams are empty.");
      }
      else {
        createSubscriber(params.default_port_value);
      }
    }
    else if(!isBlackboardPointer(bb_topic_name))
    {
      // If the content of the port "topic_name" is not
      // a pointer to the blackboard, but a static string, we can
      // create the client in the constructor.
      createSubscriber(bb_topic_name);
    }
    else {
      topic_name_may_change_ = true;
      // createSubscriber will be invoked in the first tick().
    }
  }
  else {
    if(params.default_port_value.empty()) {
      throw std::logic_error(
        "Both [topic_name] in the InputPort and the RosNodeParams are empty.");
    }
    else {
      createSubscriber(params.default_port_value);
    }
  }
}

template<class T> inline
  bool RosTopicSubNode<T>::createSubscriber(const std::string& topic_name)
{
  if(topic_name.empty())
  {
    throw RuntimeError("topic_name is empty");
  }
  
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  auto callback = std::bind(&RosTopicSubNode::topicCallback, this, std::placeholders::_1);
  subscriber_ = node_->create_subscription<T>(topic_name, 1, callback, sub_option);
  prev_topic_name_ = topic_name;
  return true;
}

template<class T> inline
  void RosTopicSubNode<T>::topicCallback(const std::shared_ptr<T> msg)
{
  last_msg_ = msg;
}

template<class T> inline
  NodeStatus RosTopicSubNode<T>::tick()
{
  // First, check if the subscriber_ is valid and that the name of the
  // topic_name in the port didn't change.
  // otherwise, create a new subscriber
  if(!subscriber_ || (status() == NodeStatus::IDLE && topic_name_may_change_))
  {
    std::string topic_name;
    getInput("topic_name", topic_name);
    if(prev_topic_name_ != topic_name)
    {
      createSubscriber(topic_name);
    }
  }

  auto CheckStatus = [](NodeStatus status)
  {
    if( !isStatusCompleted(status) )
    {
      throw std::logic_error("RosTopicSubNode: the callback must return either SUCCESS or FAILURE");
    }
    return status;
  };
  callback_group_executor_.spin_some();
  auto status = CheckStatus (onTick(last_msg_));
  last_msg_ = nullptr;

  return status;
}

}  // namespace BT

