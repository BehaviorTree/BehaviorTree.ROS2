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

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <string>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>

#include "behaviortree_ros2/ros_node_params.hpp"
#include <boost/signals2.hpp>

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
template <class TopicT>
class RosTopicSubNode : public BT::StatefulActionNode
{
public:
  // Type definitions
  using Subscriber = typename rclcpp::Subscription<TopicT>;

protected:
  struct SubscriberInstance
  {
    SubscriberInstance(std::shared_ptr<rclcpp::Node> node, const std::string& topic_name,
                       const std::size_t history_depth,
                       const rclcpp::ReliabilityPolicy& reliability);

    std::shared_ptr<Subscriber> subscriber;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor;
    boost::signals2::signal<void(const std::shared_ptr<TopicT>)> broadcaster;
    std::shared_ptr<TopicT> last_msg;
  };

  static std::mutex& registryMutex()
  {
    static std::mutex sub_mutex;
    return sub_mutex;
  }

  using SubscribersRegistry =
      std::unordered_map<std::string, std::weak_ptr<SubscriberInstance>>;

  // contains the fully-qualified name of the node and the name of the topic
  static SubscribersRegistry& getRegistry()
  {
    static SubscribersRegistry subscribers_registry;
    return subscribers_registry;
  }

  std::weak_ptr<rclcpp::Node> node_;
  std::shared_ptr<SubscriberInstance> sub_instance_;
  std::shared_ptr<TopicT> last_msg_;
  std::string topic_name_;
  boost::signals2::connection signal_connection_;
  std::string subscriber_key_;

  rclcpp::Logger logger()
  {
    if(auto node = node_.lock())
    {
      return node->get_logger();
    }
    return rclcpp::get_logger("RosTopicSubNode");
  }

public:
  /** You are not supposed to instantiate this class directly, the factory will do it.
   * To register this class into the factory, use:
   *
   *    RegisterRosAction<DerivedClass>(factory, params)
   *
   * Note that if the external_action_client is not set, the constructor will build its own.
   * */
  explicit RosTopicSubNode(const std::string& instance_name, const BT::NodeConfig& conf,
                           const RosNodeParams& params);

  virtual ~RosTopicSubNode()
  {
    signal_connection_.disconnect();
  }

  /**
   * @brief Any subclass of RosTopicNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = { InputPort<std::string>("topic_name", "__default__placeholder__",
                                               "Topic name") };
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

  NodeStatus onStart() override final;

  NodeStatus onRunning() override final;

  void onHalted() override final;

  /** Callback invoked in the tick. You must return SUCCESS, FAILURE, or RUNNING.
   *
   * @param last_msg the latest message received, since the last tick.
   *                  Will be empty if no new message received.
   *
   * @return the new status of the Node, based on last_msg
   */
  virtual NodeStatus onTick(const std::shared_ptr<TopicT>& last_msg) = 0;

  /** latch the message that has been processed. If returns false and no new message is
   * received, before next call there will be no message to process. If returns true,
   * the next call will process the same message again, if no new message received.
   *
   * This can be equated with latched vs non-latched topics in ros 1.
   *
   * @return false will clear the message after ticking/processing.
   */
  virtual bool latchLastMessage() const
  {
    return false;
  }

private:
  bool createSubscriber(const std::string& topic_name);

  NodeStatus checkStatus(const NodeStatus& status) const;
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------
template <class T>
inline RosTopicSubNode<T>::SubscriberInstance::SubscriberInstance(
    std::shared_ptr<rclcpp::Node> node, const std::string& topic_name,
    const std::size_t history_depth, const rclcpp::ReliabilityPolicy& reliability)
{
  // create a callback group for this particular instance
  callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor.add_callback_group(callback_group,
                                             node->get_node_base_interface());

  rclcpp::SubscriptionOptions option;
  option.callback_group = callback_group;

  // The callback will broadcast to all the instances of RosTopicSubNode<T>
  auto callback = [this](const std::shared_ptr<T> msg) {
    last_msg = msg;
    broadcaster(msg);
  };
  subscriber = node->create_subscription<T>(
      topic_name, rclcpp::QoS(history_depth).reliability(reliability), callback, option);
}

template <class T>
inline RosTopicSubNode<T>::RosTopicSubNode(const std::string& instance_name,
                                           const NodeConfig& conf,
                                           const RosNodeParams& params)
  : BT::StatefulActionNode(instance_name, conf), node_(params.nh)
{
  // Check if default_port_value was used to provide a topic name.
  if(!params.default_port_value.empty())
  {
    topic_name_ = params.default_port_value;
  }

  // If no value was provided through the params, assume that the topic name will be set
  // through a port when the node is first ticked.
}

template <class T>
inline bool RosTopicSubNode<T>::createSubscriber(const std::string& topic_name)
{
  if(topic_name.empty())
  {
    throw RuntimeError("topic_name is empty");
  }
  if(sub_instance_)
  {
    throw RuntimeError("Subscriber already exists");
  }

  auto node = node_.lock();
  if(!node)
  {
    throw RuntimeError("The ROS node went out of scope. RosNodeParams doesn't take the "
                       "ownership of the node.");
  }

  const auto publisher_info = node->get_publishers_info_by_topic(topic_name);
  if(publisher_info.empty())
  {
    RCLCPP_INFO(logger(),
                "No publisher found on topic [%s]. Deferring creation of subscriber "
                "until publisher exists.",
                topic_name_.c_str());
    return false;
  }

  rclcpp::ReliabilityPolicy publisher_reliability =
      publisher_info.at(0).qos_profile().reliability();
  for(std::size_t i = 1; i < publisher_info.size(); i++)
  {
    if(publisher_reliability != publisher_info.at(i).qos_profile().reliability())
    {
      RCLCPP_WARN_ONCE(logger(),
                       "Multiple publishers were found on topic [%s] with different QoS "
                       "reliability policies. The subscriber will use the reliability "
                       "setting from the first publisher it found, but this may result "
                       "in undesirable behavior.",
                       topic_name_.c_str());
      break;
    }
  }

  subscriber_key_ = std::string(node->get_fully_qualified_name()) + "/" + topic_name;

  // find SubscriberInstance in the registry
  std::unique_lock lk(registryMutex());

  auto& registry = getRegistry();
  auto it = registry.find(subscriber_key_);
  if(it == registry.end() || it->second.expired())
  {
    sub_instance_ =
        std::make_shared<SubscriberInstance>(node, topic_name, 1, publisher_reliability);
    registry.insert({ subscriber_key_, sub_instance_ });

    RCLCPP_INFO(logger(), "Node [%s] created Subscriber to topic [%s]", name().c_str(),
                topic_name.c_str());
  }
  else
  {
    sub_instance_ = it->second.lock();
  }

  // Check if there was a message received before the creation of this subscriber action
  if(sub_instance_->last_msg)
  {
    last_msg_ = sub_instance_->last_msg;
  }

  // add "this" as received of the broadcaster
  signal_connection_ = sub_instance_->broadcaster.connect(
      [this](const std::shared_ptr<T> msg) { last_msg_ = msg; });

  return true;
}
template <class T>
inline NodeStatus RosTopicSubNode<T>::checkStatus(const NodeStatus& status) const
{
  if(!isStatusActive(status))
  {
    throw std::logic_error("RosTopicSubNode: the callback must return SUCCESS, FAILURE, "
                           "or RUNNING");
  }
  return status;
};

template <class T>
inline NodeStatus RosTopicSubNode<T>::onStart()
{
  // If the topic name was provided through an input port, is a valid topic name, and is different
  // from the stored topic name, update the stored topic name to the new string.
  if(const auto topic_name = getInput<std::string>("topic_name");
     topic_name.has_value() && !topic_name->empty() &&
     topic_name.value() != "__default__placeholder__" && topic_name != topic_name_)
  {
    topic_name_ = topic_name.value();
  }

  // If we fail to create a subscriber, exit early and try again on the next tick
  if(!createSubscriber(topic_name_))
  {
    return NodeStatus::RUNNING;
  }

  // Discard any previous message unless latching is enabled.
  // This enforces that the message must have been received after the current call to onStart(), even if this node has been ticked to completion previously.
  if(!latchLastMessage())
  {
    last_msg_.reset();
  }

  // NOTE(schornakj): Something weird is happening here that prevents the subscriber from receiving a message if the publisher both initializes and publishes a message in between ticks.
  // I think it has to do with the discovery process between the publisher and subscriber and how that relates to events being handled by the executor.
  // This might depend on the middleware implemenation too.

  sub_instance_->callback_group_executor.spin_some();

  // If no message was received, return RUNNING
  if(last_msg_ == nullptr)
  {
    return NodeStatus::RUNNING;
  }

  auto status = checkStatus(onTick(last_msg_));
  if(!latchLastMessage())
  {
    last_msg_.reset();
  }
  return status;
}

template <class T>
inline NodeStatus RosTopicSubNode<T>::onRunning()
{
  // If the subscriber wasn't initialized during onStart(), attempt to create it here.
  if(!sub_instance_)
  {
    // If the subscriber still cannot be created, return RUNNING and try again on the next tick.
    if(!createSubscriber(topic_name_))
    {
      return NodeStatus::RUNNING;
    }
  }

  sub_instance_->callback_group_executor.spin_some();

  // If no message was received, return RUNNING
  if(last_msg_ == nullptr)
  {
    return NodeStatus::RUNNING;
  }

  // TODO(schornakj): handle timeout here

  auto status = checkStatus(onTick(last_msg_));

  return status;
}

template <class T>
inline void RosTopicSubNode<T>::onHalted()
{}
}  // namespace BT
