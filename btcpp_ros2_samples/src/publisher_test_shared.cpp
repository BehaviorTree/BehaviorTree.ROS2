#include "behaviortree_ros2/bt_topic_pub_shared_node.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>

using namespace BT;

class SendString: public RosTopicPubSharedNode<std_msgs::msg::String, SendString>
{
public:
  SendString(const std::string& name,
             const NodeConfig& conf,
             const RosNodeParams& params)
    : RosTopicPubSharedNode<std_msgs::msg::String, SendString>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    BT::PortsList addition;
    BT::PortsList basic = {
      InputPort<std::string>("message", "Hello World!", "Message to send")
    };
    basic.insert(addition.begin(), addition.end());

    return providedBasicPorts(basic);
  }

  bool setMessage(std_msgs::msg::String& msg)
  {
    // RCLCPP_INFO(rclcpp::get_logger("test"), "publisher pointer: %p", reinterpret_cast<void*>(publisher_.get()));
    std::string user_message = "";
    getInput("message", user_message);
    msg.data = user_message;
    return true;
  }
};

  // Simple tree, used to execute once each action.
  static const char* xml_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <SendString topic_name="A" message="call 1"/>
        <SendString topic_name="A" message="call 2"/>

      </Sequence>
    </BehaviorTree>
  </root>
 )";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("publisher_shared_test");

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "btcpp_string";
  factory.registerNodeType<SendString>("SendString", params);

  auto tree = factory.createTreeFromText(xml_text);

  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }
  return 0;
}
