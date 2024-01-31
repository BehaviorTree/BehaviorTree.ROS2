#include "behaviortree_ros2/bt_topic_sub_shared_node.hpp"
#include <std_msgs/msg/string.hpp>

using namespace BT;

class ReceiveString: public RosTopicSubSharedNode<std_msgs::msg::String, ReceiveString>
{
public:
  ReceiveString(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicSubSharedNode<std_msgs::msg::String, ReceiveString>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    BT::PortsList addition;
    BT::PortsList basic = {};
    basic.insert(addition.begin(), addition.end());

    return providedBasicPorts(basic);
  }

  NodeStatus onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg) override
  {
    // RCLCPP_INFO(rclcpp::get_logger("test"), "%s, subscriber pointer: %p", name().c_str(), reinterpret_cast<void*>(sub_instance_->subscriber.get()));
    if(last_msg) // empty if no new message received, since the last tick
    {
      RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(), last_msg->data.c_str());
    }
    return NodeStatus::SUCCESS;
  }
};

  // Simple tree, used to execute once each action.
  static const char* xml_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <ReceiveString name="A" topic_name="A"/>
        <Delay delay_msec="4000">
          <AlwaysSuccess/>
        </Delay>
        <ReceiveString name="B" topic_name="A"/>
      </Sequence>
    </BehaviorTree>
  </root>
 )";

        // <ReceiveString name="B" topic_name="A"/>
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("subscriber_shared_test");

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "btcpp_string";
  factory.registerNodeType<ReceiveString>("ReceiveString", params);

  auto tree = factory.createTreeFromText(xml_text);

  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }

  return 0;
}
