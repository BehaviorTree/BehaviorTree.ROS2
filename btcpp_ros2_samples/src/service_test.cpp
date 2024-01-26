#include "behaviortree_ros2/bt_service_node.hpp"
#include "btcpp_ros2_interfaces/srv/add_two_ints.hpp"

using namespace BT;

class AddTwoIntsClient: public RosServiceNode<btcpp_ros2_interfaces::srv::AddTwoInts>
{
public:
  AddTwoIntsClient(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<btcpp_ros2_interfaces::srv::AddTwoInts>(name, conf, params)
  {}

  static PortsList providedPorts()
  {
    BT::PortsList addition;
    BT::PortsList basic = {
      InputPort<int>("a", 12, "first integer to add"),
      InputPort<int>("b", 12, "second integer to add")
    };
    basic.insert(addition.begin(), addition.end());
    return providedBasicPorts(basic);
  }

  bool setRequest(Request::SharedPtr& req) override
  {
    getInput("a", a);
    getInput("b", b);
    req->a = a;
    req->b = b;

    return true;
  }

  virtual BT::NodeStatus onResponseReceived(const Response::SharedPtr& res) override
  {
    RCLCPP_INFO(rclcpp::get_logger("test"), "pointer: %p", reinterpret_cast<void*>(service_client_.get()));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result of %d + %d = %ld",
               a, b, res->sum);
    return BT::NodeStatus::SUCCESS;
  }

  virtual BT::NodeStatus onFailure(ServiceNodeErrorCode error_code) override
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "AddTwoIntsClient failed with error code %d", error_code);
    return BT::NodeStatus::FAILURE;
  }
private:
  int a;
  int b;
};

  // Simple tree, used to execute once each action.
  static const char* xml_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <AddTwoIntsClient service_name="add_two_ints" a="3" b="4"/>
        <AddTwoIntsClient service_name="add_two_ints" a="10" b="4"/>
      </Sequence>
    </BehaviorTree>
  </root>
 )";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("service_test");

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "btcpp_service";
  factory.registerNodeType<AddTwoIntsClient>("AddTwoIntsClient", params);

  auto tree = factory.createTreeFromText(xml_text);

  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }
  return 0;
}
