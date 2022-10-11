#include <behaviortree_ros2/bt_service_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"


using namespace BT;

//-------------------------------------------------------------
// Simple Action to print a number
//-------------------------------------------------------------

class PrintValue : public BT::SyncActionNode
{
public:
  PrintValue(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    std::string msg;
    if( getInput("message", msg ) ){
      std::cout << "PrintValue: " << msg << std::endl;
      return NodeStatus::SUCCESS;
    }
    else{
      std::cout << "PrintValue FAILED "<< std::endl;
      return NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts() {
    return{ BT::InputPort<std::string>("message") };
  }
};

class SleepService: public RosServiceNode<example_interfaces::srv::AddTwoInts>
{
public:
  SleepService(const std::string& name,
              const BT::NodeConfiguration& conf,
              const ServiceNodeParams& params,
              typename std::shared_ptr<ServiceClient> service_client)
    : RosServiceNode<example_interfaces::srv::AddTwoInts>(name, conf, params, service_client)
  {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<unsigned>("msec"),
      InputPort<int>("first_int"),
      InputPort<int>("second_int"),
      OutputPort<int>("sum") };
  }

  void sendRequest(typename ServiceType::Request::SharedPtr request) override
  {

    getInput("first_int", request->a);
    
    getInput("second_int", request->b);
    expected_result_ = request->a + request->b;
    RCLCPP_INFO(node_->get_logger(),"AddTwoInts: sending request");
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    RCLCPP_INFO(node_->get_logger(),"AddTwoInts: response received");
    if( rep->sum == expected_result_)
    {
      setOutput<int>("sum", rep->sum);
      return NodeStatus::SUCCESS;
    }
    else{
      RCLCPP_ERROR(node_->get_logger(),"AddTwoInts replied something unexpected: %d", rep->sum);
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(ServiceNodeErrorCode failure) override
  {
    RCLCPP_ERROR(node_->get_logger(),"AddTwoInts request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

private:
  int expected_result_;


};


//-----------------------------------------------------

  // Simple tree, used to execute once each action.
  static const char* xml_text = R"(
 <root >
     <BehaviorTree>
        <Sequence>
            <PrintValue message="start"/>
            <Sleep first_int='1' second_int='2' msec="2000"/>
            <PrintValue message="sleep completed"/>
            <Fallback>
                <Timeout msec="500">
                   <Sleep first_int='15' second_int='12' msec="1000"/>
                </Timeout>
                <PrintValue message="sleep aborted"/>
            </Fallback>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("sleep_client");
  

  BehaviorTreeFactory factory;

  factory.registerNodeType<PrintValue>("PrintValue");

  ServiceNodeParams params = {nh, "add_two_ints", std::chrono::milliseconds(10000)};
  RegisterRosService<SleepService>(factory, "Sleep", params);

  auto tree = factory.createTreeFromText(xml_text);

  // This logger publish status changes using ZeroMQ. Used by Groot
  PublisherZMQ publisher_zmq(tree);

  NodeStatus status = NodeStatus::IDLE;

  while( rclcpp::ok() )
  {
    status = tree.tickRoot();
    tree.sleep(std::chrono::milliseconds(100));
  }

  return 0;
}
