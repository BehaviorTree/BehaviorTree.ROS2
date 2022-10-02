#include <behaviortree_ros2/bt_action_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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

class SendGoalAction: public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  SendGoalAction(const std::string& name,
              const BT::NodeConfiguration& conf,
              const ActionNodeParams& params,
              typename std::shared_ptr<ActionClient> action_client)
    : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params, action_client)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("x"),BT::InputPort<double>("y"),BT::InputPort<double>("yaw")};
  }

  bool setGoal(Goal& goal) override
  {
    double x,y,yaw ;
    getInput<double>("x",x);
    getInput<double>("y",y);
    getInput<double>("yaw",yaw);
    
    geometry_msgs::msg::PoseStamped goal_pose_;
    goal_pose_.header.frame_id = "/map";
    goal_pose_.pose.position.x = x;
    goal_pose_.pose.position.y = y;
    goal_pose_.pose.position.z = 0.0;
    goal_pose_.pose.orientation.x = 0.0;
    goal_pose_.pose.orientation.y = 0.0;
    goal_pose_.pose.orientation.z = std::sin(yaw/2);
    goal_pose_.pose.orientation.w = std::cos(yaw/2);

    goal.pose = goal_pose_;
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    RCLCPP_INFO( node_->get_logger(), "onResultReceived %d", wr.result );
    return wr.result ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
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
            <Fallback>
                <SendGoalAction x="0" y="0" yaw="0"/>
                <PrintValue message="Goal aborted"/>
            </Fallback>
            <PrintValue message="Goal completed"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("nav2_client");
  

  BehaviorTreeFactory factory;

  factory.registerNodeType<PrintValue>("PrintValue");

  ActionNodeParams params = {nh, "navigate_to_pose", std::chrono::milliseconds(2000)};
  RegisterRosAction<SendGoalAction>(factory, "SendGoalAction", params);

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
