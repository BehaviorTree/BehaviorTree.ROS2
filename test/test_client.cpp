#include <behaviortree_ros2/bt_action_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include "behaviortree_ros2/action/sleep.hpp"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
// #include <behaviortree_ros2/bt_service_node.hpp>


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

class SleepAction: public RosActionNode<behaviortree_ros2::action::Sleep>
{
public:
  SleepAction(const std::string& name,
              const BT::NodeConfiguration& conf,
              const ActionNodeParams& params,
              typename std::shared_ptr<ActionClient> action_client)
    : RosActionNode<behaviortree_ros2::action::Sleep>(name, conf, params, action_client)
  {}

  static BT::PortsList providedPorts()
  {
    return {InputPort<unsigned>("msec")};
  }

  bool setGoal(Goal& goal) override
  {
    auto timeout = getInput<unsigned>("msec");
    goal.msec_timeout = timeout.value();
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    RCLCPP_INFO( node_->get_logger(), "onResultReceived %d", wr.result->done );
    return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR( node_->get_logger(), "onFailure %d", error );
    return NodeStatus::FAILURE;
  }
};


//-----------------------------------------------------

  // Simple tree, used to execute once each action.
  static const char* xml_text = R"(
 <root >
     <BehaviorTree>
        <Sequence>
            <PrintValue message="start"/>
            <Sleep msec="2000"/>
            <PrintValue message="sleep completed"/>
            <Fallback>
                <Timeout msec="500">
                   <Sleep msec="1000"/>
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

  ActionNodeParams params = {nh, "sleep_service", std::chrono::milliseconds(2000)};
  RegisterRosAction<SleepAction>(factory, "Sleep", params);

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
