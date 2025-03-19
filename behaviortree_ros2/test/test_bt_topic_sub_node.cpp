#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/empty.hpp>

namespace
{
constexpr auto kTopicName = "/my_topic";
constexpr auto kHistoryDepth = 1;
constexpr auto kPortIdTopicName = "topic_name";

using Empty = std_msgs::msg::Empty;
}  // namespace

namespace BT
{

class SubNode : public RosTopicSubNode<Empty>
{
public:
  SubNode(const std::string& instance_name, const BT::NodeConfig& conf,
          const RosNodeParams& params)
    : RosTopicSubNode<Empty>(instance_name, conf, params)
  {}

private:
  NodeStatus onTick(const std::shared_ptr<Empty>& last_msg) override
  {
    if(last_msg == nullptr)
    {
      return NodeStatus::RUNNING;
    }
    return NodeStatus::SUCCESS;
  }
};

class TestBtTopicSubNode : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>("ros_node");

    BT::assignDefaultRemapping<SubNode>(config_);
    config_.blackboard = Blackboard::create();
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  void createPublisher(const rclcpp::QoS& qos)
  {
    publisher_ = node_->create_publisher<Empty>(kTopicName, qos);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Publisher<Empty>> publisher_;

  NodeConfig config_;
};

TEST_F(TestBtTopicSubNode, TopicAsParam)
{
  // GIVEN the blackboard does not contain the topic name

  // GIVEN the input params contain the topic name
  RosNodeParams params;
  params.nh = node_;
  params.default_port_value = kTopicName;

  // GIVEN we pass in the params and the blackboard when creating the BT node
  SubNode bt_node("test_node", config_, params);

  // GIVEN we create publisher with default QoS settings after creating the BT node
  createPublisher(rclcpp::QoS(kHistoryDepth));

  // GIVEN the BT node is RUNNING before the publisher publishes a message
  ASSERT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::RUNNING));

  // GIVEN the publisher has published a message
  publisher_->publish(std_msgs::build<Empty>());

  // WHEN the BT node is ticked
  // THEN it succeeds
  EXPECT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::SUCCESS));
}

TEST_F(TestBtTopicSubNode, TopicFromStaticStringInPort)
{
  // GIVEN the input port directly contains the topic name
  config_.input_ports[kPortIdTopicName] = kTopicName;

  // GIVEN the input params do not contain the topic name
  RosNodeParams params;
  params.nh = node_;

  // GIVEN we pass in the params and the blackboard when creating the BT node
  SubNode bt_node("test_node", config_, params);

  // GIVEN we create publisher with default QoS settings after creating the BT node
  createPublisher(rclcpp::QoS(kHistoryDepth));

  // GIVEN the BT node is RUNNING before the publisher publishes a message
  ASSERT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::RUNNING));

  // GIVEN the publisher has published a message
  publisher_->publish(std_msgs::build<Empty>());

  // WHEN the BT node is ticked
  // THEN it succeeds
  EXPECT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::SUCCESS));
}

TEST_F(TestBtTopicSubNode, TopicFromBlackboard)
{
  // GIVEN the input port is remapped to point to a value on the blackboard
  config_.blackboard->set(kPortIdTopicName, kTopicName);

  // GIVEN the input params do not contain the topic name
  RosNodeParams params;
  params.nh = node_;

  // GIVEN we pass in the params and the blackboard when creating the BT node
  SubNode bt_node("test_node", config_, params);

  // GIVEN we create publisher with default QoS settings after creating the BT node
  createPublisher(rclcpp::QoS(kHistoryDepth));

  // GIVEN the BT node is RUNNING before the publisher publishes a message
  ASSERT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::RUNNING));

  // GIVEN the publisher has published a message
  publisher_->publish(std_msgs::build<Empty>());

  // WHEN the BT node is ticked
  // THEN it succeeds
  EXPECT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::SUCCESS));
}

TEST_F(TestBtTopicSubNode, QoSBestEffort)
{
  // GIVEN the blackboard does not contain the topic name

  // GIVEN the input params contain the topic name
  RosNodeParams params;
  params.nh = node_;
  params.default_port_value = kTopicName;

  // GIVEN we pass in the params and the blackboard when creating the BT node
  SubNode bt_node("test_node", config_, params);

  // GIVEN we create publisher with BestEffort reliability QoS settings after creating the BT node
  createPublisher(rclcpp::QoS(kHistoryDepth).best_effort());

  // GIVEN the BT node is RUNNING before the publisher publishes a message
  ASSERT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::RUNNING));

  // GIVEN the publisher has published a message
  publisher_->publish(std_msgs::build<Empty>());

  // WHEN the BT node is ticked
  // THEN it succeeds
  EXPECT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::SUCCESS));
}

TEST_F(TestBtTopicSubNode, PublisherCreatedAfterFirstTick)
{
  // GIVEN the blackboard does not contain the topic name

  // GIVEN the input params contain the topic name
  RosNodeParams params;
  params.nh = node_;
  params.default_port_value = kTopicName;

  // GIVEN we pass in the params and the blackboard when creating the BT node
  SubNode bt_node("test_node", config_, params);

  // GIVEN the BT node is RUNNING before the publisher is created
  ASSERT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::RUNNING));

  // GIVEN we create publisher with Reliable reliability QoS settings after creating the BT node and after the node starts ticking
  createPublisher(rclcpp::QoS(kHistoryDepth).reliable());

  // TODO(Joe): Why does the node need to be ticked in between the publisher appearing and sending a message for the message to be received? Seems highly suspicious!
  ASSERT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::RUNNING));

  // GIVEN the publisher has published a message
  publisher_->publish(std_msgs::build<Empty>());
  // WHEN the BT node is ticked
  // THEN it succeeds
  EXPECT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::SUCCESS));
}

TEST_F(TestBtTopicSubNode, ExceptionIfNoTopic)
{
  // GIVEN the blackboard does not contain the topic name

  // GIVEN the input params do not contain the topic name
  RosNodeParams params;
  params.nh = node_;

  // GIVEN we pass in the params and the blackboard when creating the BT node
  SubNode bt_node("test_node", config_, params);

  // WHEN the BT node is ticked
  // THEN it throws an exception
  EXPECT_THROW(bt_node.executeTick(), BT::RuntimeError);
}

TEST_F(TestBtTopicSubNode, ExceptionIfRosNodeReset)
{
  // GIVEN the blackboard does not contain the topic name

  // GIVEN the input params do not contain the topic name
  RosNodeParams params;
  params.nh = node_;

  // GIVEN we pass in the params and the blackboard when creating the BT node
  SubNode bt_node("test_node", config_, params);

  // GIVEN the ROS node is reset prior to the first tick
  node_.reset();

  // WHEN the BT node is ticked
  // THEN it throws an exception
  EXPECT_THROW(bt_node.executeTick(), BT::RuntimeError);
}
}  // namespace BT
