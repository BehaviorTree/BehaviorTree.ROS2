#ifndef BEHAVIOR_TREE_ROS2__BT_TOPIC_NODE_HPP_
#define BEHAVIOR_TREE_ROS2__BT_TOPIC_NODE_HPP_

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
class RosTopicNode : public BT::ConditionNode
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
  explicit RosTopicNode(const std::string & instance_name,
                         const BT::NodeConfig& conf,
                         const NodeParams& params);

  virtual ~RosTopicNode() = default;

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
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

  /** topicCallback is a callback invoked when a message is received.
   */
  void topicCallback(const std::shared_ptr<TopicT> msg);

  /** Callback invoked when a message is received.
   * It is up to the user to define if the BT::action returns SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onMessageReceived(const typename TopicT::SharedPtr& last_msg) = 0;

  /** Callback invoked when a message is not received.
   * It is up to the user to define if the BT::action returns SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onMessageNotReceived()
  {
    return BT::NodeStatus::FAILURE;
  }

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
  RosTopicNode<T>::RosTopicNode(const std::string & instance_name,
                                  const NodeConfig &conf,
                                  const NodeParams& params)
    : BT::ConditionNode(instance_name, conf),
      node_(params.nh)
{ 
  // Three cases:
  // - we use the default topic_name in NodeParams when port is empty
  // - we use the topic_name in the port and it is a static string.
  // - we use the topic_name in the port and it is blackboard entry.
  
  // check port remapping
  auto portIt = config().input_ports.find("topic_name");
  if(portIt != config().input_ports.end())
  {
    const std::string& bb_topic_name = portIt->second;

    if(bb_topic_name.empty() || bb_topic_name == "__default__placeholder__")
    {
      if(params.default_server_name.empty()) {
        throw std::logic_error(
          "Both [topic_name] in the InputPort and the NodeParams are empty.");
      }
      else {
        createSubscriber(params.default_server_name);
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
    if(params.default_server_name.empty()) {
      throw std::logic_error(
        "Both [topic_name] in the InputPort and the NodeParams are empty.");
    }
    else {
      createSubscriber(params.default_server_name);
    }
  }
}

template<class T> inline
  bool RosTopicNode<T>::createSubscriber(const std::string& topic_name)
{
  if(topic_name.empty())
  {
    throw RuntimeError("topic_name is empty");
  }
  
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  subscriber_ = node_->create_subscription<T>(topic_name, 1, std::bind(&RosTopicNode::topicCallback, this, std::placeholders::_1), sub_option);
  prev_topic_name_ = topic_name;
  return true;
}

template<class T> inline
  void RosTopicNode<T>::topicCallback(const std::shared_ptr<T> msg)
{
  last_msg_ = msg;
}

template<class T> inline
  NodeStatus RosTopicNode<T>::tick()
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
    if( status != NodeStatus::SUCCESS && status != NodeStatus::FAILURE )
    {
      throw std::logic_error("RosTopicNode: the callback must return either SUCCESS or FAILURE");
    }
    return status;
  };
  last_msg_ = nullptr;
  callback_group_executor_.spin_some();
  if (last_msg_) {
    return CheckStatus (onMessageReceived(last_msg_));
  }
  return CheckStatus (onMessageNotReceived());
}

}  // namespace BT

#endif  // BEHAVIOR_TREE_ROS2__BT_TOPIC_NODE_HPP_
