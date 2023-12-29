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
#include <boost/signals2.hpp>

namespace BT
{
/**
 * @brief Settings for how the subscriber should read messages
 * 
 * READ_ONCE: Read the message once then clear it
 * READ_LATCH: Keep reading the same message until a new one is received
 * READ_CONFIGURABLE: Read the message once then clear it, unless the "read_last" port is set to true 
 */
enum class SubscriberReadMode { READ_ONCE, READ_LATCH, READ_CONFIGURABLE };

/**
 * @brief Abstract class to wrap a Topic subscriber.
 * Considering the example in the tutorial:
 * https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
 *
 * The corresponding wrapper would be:
 *
 * class SubscriberNode: RosTopicSubNode<std_msgs::msg::String, SubscriberReadMode::READ_ONCE>
 *
 * The name of the topic will be determined as follows:
 *
 * 1. If a value is passes in the InputPort "topic_name", use that
 * 2. Otherwise, use the value in RosNodeParams::default_port_value
 * 
 * To configure the reading mode, set the 2nd class template parameter to one of the options in SubscriberReadMode.
 * If not set READ_ONCE will be used by default for backwards compatibility.
 */
template<class TopicT, SubscriberReadMode read_mode = SubscriberReadMode::READ_ONCE>
class RosTopicSubNode : public BT::ConditionNode
{
 public:
  // Type definitions
  using Subscriber = typename rclcpp::Subscription<TopicT>;

 protected: 
  struct SubscriberInstance
  {
    void init(std::shared_ptr<rclcpp::Node> node, const std::string& topic_name)
    {
      // create a callback group for this particular instance
      callback_group = 
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
      callback_group_executor.add_callback_group(
        callback_group, node->get_node_base_interface());

      rclcpp::SubscriptionOptions option;
      option.callback_group = callback_group;

    // The callback will broadcast to all the instances of RosTopicSubNode<T>
      auto callback = [this](const std::shared_ptr<TopicT> msg) 
      {
        broadcaster(msg);
      };
      subscriber =  node->create_subscription<TopicT>(topic_name, 1, callback, option);
    }

    std::shared_ptr<Subscriber> subscriber;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor;
    boost::signals2::signal<void (const std::shared_ptr<TopicT>)> broadcaster;


  };

  static std::mutex& registryMutex()
  {
    static std::mutex sub_mutex;
    return sub_mutex;
  }

  // contains the fully-qualified name of the node and the name of the topic
  static SubscriberInstance& getRegisteredInstance(const std::string& key)
  {
    static std::unordered_map<std::string, SubscriberInstance> subscribers_registry;
    return subscribers_registry[key];
  }

  std::shared_ptr<rclcpp::Node> node_;
  SubscriberInstance* sub_instance_ = nullptr;
  std::shared_ptr<TopicT> last_msg_;
  std::string topic_name_;
  boost::signals2::connection signal_connection_;

  rclcpp::Logger logger()
  {
    return node_->get_logger();
  }

 public:

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
    PortsList basic = {
      InputPort<std::string>("topic_name", "__default__placeholder__", "Topic name")
    };
    if (read_mode == SubscriberReadMode::READ_CONFIGURABLE)
    {
      basic.insert(InputPort<bool>(
        "read_last",
        false,
        "Read mode. True if read last message, even if it has already been seen. False if read only new messages."));
    }
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

  /** Callback invoked in the tick. You must return either SUCCESS of FAILURE
   *
   * @param last_msg the latest message received, since the last tick.
   *                  Will be empty if no new message received.
   * 
   * @return the new status of the Node, based on last_msg
   */
  virtual NodeStatus onTick(const std::shared_ptr<TopicT>& last_msg) = 0;

private:

  bool createSubscriber(const std::string& topic_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template<class T, SubscriberReadMode read_mode = SubscriberReadMode::READ_ONCE> inline
  RosTopicSubNode<T, read_mode>::RosTopicSubNode(const std::string & instance_name,
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
      // do nothing
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

template<class T, SubscriberReadMode read_mode> inline
  bool RosTopicSubNode<T, read_mode>::createSubscriber(const std::string& topic_name)
{
  if(topic_name.empty())
  {
    throw RuntimeError("topic_name is empty");
  }
  if(sub_instance_)
  {
    throw RuntimeError("Can't call createSubscriber more than once");
  }

  // find SubscriberInstance in the registry
  std::unique_lock lk(registryMutex());
  const auto key = topic_name + "@" + node_->get_fully_qualified_name();
  sub_instance_ = &(getRegisteredInstance(key));

  // just created (subscriber is not initialized)
  if(!sub_instance_->subscriber)
  {
    sub_instance_->init(node_, topic_name);

    RCLCPP_INFO(logger(), 
      "Node [%s] created Subscriber to topic [%s]",
      name().c_str(), topic_name.c_str() );
  }

  // add "this" as received of the broadcaster
  signal_connection_ = sub_instance_->broadcaster.connect(
    [this](const std::shared_ptr<T> msg) { last_msg_ = msg; } );

  topic_name_ = topic_name;
  return true;
}


template<class T, SubscriberReadMode read_mode> inline
  NodeStatus RosTopicSubNode<T, read_mode>::tick()
{
  // First, check if the subscriber_ is valid and that the name of the
  // topic_name in the port didn't change.
  // otherwise, create a new subscriber
  if(!sub_instance_)
  {
    std::string topic_name;
    getInput("topic_name", topic_name);
    createSubscriber(topic_name); 
  }

  auto CheckStatus = [](NodeStatus status)
  {
    if( !isStatusCompleted(status) )
    {
      throw std::logic_error("RosTopicSubNode: the callback must return" 
                             "either SUCCESS or FAILURE");
    }
    return status;
  };
  sub_instance_->callback_group_executor.spin_some();
  auto status = CheckStatus (onTick(last_msg_));
  if (read_mode == SubscriberReadMode::READ_ONCE ||
    (read_mode == SubscriberReadMode::READ_CONFIGURABLE && !getInput<bool>("read_last").value()))
  {
    last_msg_ = nullptr;
  }

  return status;
}

}  // namespace BT

