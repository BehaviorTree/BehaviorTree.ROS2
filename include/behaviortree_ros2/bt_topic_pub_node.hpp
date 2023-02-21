#ifndef BEHAVIOR_TREE_ROS2__BT_TOPIC_PUB_NODE_HPP_
#define BEHAVIOR_TREE_ROS2__BT_TOPIC_PUB_NODE_HPP_

#include <memory>
#include <string>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/node_params.hpp"

namespace BT
{

/**
 * @brief Abstract class representing use to call a ROS2 Service (client).
 *
 * It will try to be non-blocking for the entire duration of the call.
 * The derived class whould reimplement the virtual methods as described below.
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
                         const NodeParams& params);

  virtual ~RosTopicPubNode() = default;

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = {
      InputPort<std::string>("topic_name", "default", "Topic name")
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

  virtual bool setMessage(TopicT& msg) = 0;

protected:

  std::shared_ptr<rclcpp::Node> node_;
  std::string prev_topic_name_;
  bool topic_name_may_change_ = false;

private:

  bool createPublisher(const std::string& topic_name);

  std::shared_ptr<Publisher> publisher_;
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template<class T> inline
  RosTopicPubNode<T>::RosTopicPubNode(const std::string & instance_name,
                                  const NodeConfig &conf,
                                  const NodeParams& params)
    : BT::ConditionNode(instance_name, conf),
      node_(params.nh)
{ 
  // If the content of the port "action_name" is not
  // a pointer to the blackboard, but a static string, we can
  // create the client in the constructor.
  const std::string topic_name = config().input_ports.at("topic_name");    
  if(topic_name == "default")
  {
    if(params.default_server_name.empty())
    {
      throw RuntimeError("Both default_server_name  and topic_name is empty");
    }
    createPublisher(params.default_server_name);
  }
  else if(!isBlackboardPointer(topic_name))
  {
    createPublisher(topic_name);
  }
  else {
    topic_name_may_change_ = true;
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

#endif  // BEHAVIOR_TREE_ROS2__BT_TOPIC_PUB_NODE_HPP_
