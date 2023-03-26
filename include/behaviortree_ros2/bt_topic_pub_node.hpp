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
 * @brief Abstract class to wrap a ROS publisher
 *
 */
template<class TopicT>
class RosTopicPubNode : public BT::ConditionNode
{

public:
  // Type definitions
  using Publisher = typename rclcpp::Publisher<TopicT>;

  /** You are not supposed to instantiate this class directly, the factory will do it.
   * To register this class into the factory, use:
   *
   *    RegisterRosAction<DerivedClasss>(factory, params)
   *
   * Note that if the external_action_client is not set, the constructor will build its own.
   * */
  explicit RosTopicPubNode(const std::string & instance_name,
                           const BT::NodeConfig& conf,
                           const RosNodeParams& params);

  virtual ~RosTopicPubNode() = default;

  /**
   * @brief Any subclass of RosTopicPubNode that has additinal ports must provide a
   * providedPorts method and call providedBasicPorts in it.
   *
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

  /**
   * @brief setMessage is a callback invoked in tick to allow the user to pass
   * the message to be published.
   *
   * @param msg the message.
   * @return  return false if anything is wrong and we must not send the message.
   * the Condition will return FAILURE.
   */
  virtual bool setMessage(TopicT& msg) = 0;

protected:

  std::shared_ptr<rclcpp::Node> node_;
  std::string prev_topic_name_;
  bool topic_name_may_change_ = false;

private:

  std::shared_ptr<Publisher> publisher_;

  bool createPublisher(const std::string& topic_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template<class T> inline
  RosTopicPubNode<T>::RosTopicPubNode(const std::string & instance_name,
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
        createPublisher(params.default_port_value);
      }
    }
    else if(!isBlackboardPointer(bb_topic_name))
    {
      // If the content of the port "topic_name" is not
      // a pointer to the blackboard, but a static string, we can
      // create the client in the constructor.
      createPublisher(bb_topic_name);
    }
    else {
      topic_name_may_change_ = true;
      // createPublisher will be invoked in the first tick().
    }
  }
  else {
    if(params.default_port_value.empty()) {
      throw std::logic_error(
        "Both [topic_name] in the InputPort and the RosNodeParams are empty.");
    }
    else {
      createPublisher(params.default_port_value);
    }
  }
}

template<class T> inline
  bool RosTopicPubNode<T>::createPublisher(const std::string& topic_name)
{
  if(topic_name.empty())
  {
    throw RuntimeError("topic_name is empty");
  }
  
  publisher_ = node_->create_publisher<T>(topic_name, 1);
  prev_topic_name_ = topic_name;
  return true;
}

template<class T> inline
  NodeStatus RosTopicPubNode<T>::tick()
{
  // First, check if the subscriber_ is valid and that the name of the
  // topic_name in the port didn't change.
  // otherwise, create a new subscriber
  if(!publisher_ || (status() == NodeStatus::IDLE && topic_name_may_change_))
  {
    std::string topic_name;
    getInput("topic_name", topic_name);
    if(prev_topic_name_ != topic_name)
    {
      createPublisher(topic_name);
    }
  }

  T msg;
  if (!setMessage(msg))
  {
    return NodeStatus::FAILURE;
  }
  publisher_->publish(msg);
  return NodeStatus::SUCCESS;
}

}  // namespace BT

