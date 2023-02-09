#ifndef BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_
#define BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_

#include <memory>
#include <string>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/node_params.hpp"

namespace BT
{

// enum ServiceNodeErrorCode
// {
//   SERVICE_UNREACHABLE,
//   SERVICE_TIMEOUT,
//   INVALID_REQUEST,
//   SERVICE_ABORTED
// };

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
  static PortsList providedBasicPorts(PortsList addition);

  /**
   * @brief Creates list of BT ports
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedPorts();

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
  std::string topic_name_;

private:

  std::shared_ptr<Subscriber> subscriber_;
  typename TopicT::SharedPtr last_msg_;
  bool is_message_received_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template<class T> inline
  RosTopicNode<T>::RosTopicNode(const std::string & instance_name,
                                  const NodeConfig &conf,
                                  const NodeParams& params)
    : BT::ConditionNode(instance_name, conf),
      node_(params.nh),
      topic_name_(params.server_name)
{
  std::string remapped_topic_name;
  if (getInput("server_name", remapped_topic_name)) {
    topic_name_ = remapped_topic_name;
  }
  std::string node_namespace = node_->get_namespace();
  // Append namespace to the service name
  if(node_namespace != "/" && topic_name_.front() != '/') {
    topic_name_ = node_namespace + "/" + topic_name_;
  }
  auto callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group;
  subscriber_ = node_->create_subscription<T>(topic_name_, 1, std::bind(&RosTopicNode::topicCallback, this, std::placeholders::_1), sub_option);
}

template<class T> inline
  PortsList RosTopicNode<T>::providedBasicPorts(PortsList addition)
{
  PortsList basic = {
    InputPort<std::string>("server_name", "Topic name")
  };
  basic.insert(addition.begin(), addition.end());
  return basic;
}

template<class T> inline
  PortsList RosTopicNode<T>::providedPorts()
{
  return providedBasicPorts({});
}

template<class T> inline
  void RosTopicNode<T>::topicCallback(const std::shared_ptr<T> msg)
{
  last_msg_ = msg;
}

template<class T> inline
  NodeStatus RosTopicNode<T>::tick()
{
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

#endif  // BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_
