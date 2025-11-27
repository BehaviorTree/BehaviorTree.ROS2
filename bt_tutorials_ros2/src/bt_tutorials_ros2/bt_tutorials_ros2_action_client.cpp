#include "bt_tutorials_ros2/bt_tutorials_ros2_action_client.hpp"

namespace bt_tutorials_ros2_action_client
{

FibonacciActionNode::FibonacciActionNode(const std::string &name, const BT::NodeConfig &conf,
                                         const BT::RosNodeParams &params)
    : BT::RosActionNode<Fibonacci>(name, conf, params)
{
    // Convert weak_ptr (from ROS params) to shared_ptr for usage in ROS-related tasks like logging
    auto shared_node = params.nh.lock();
    if (!shared_node)
    {
        throw std::runtime_error("FibonacciActionNode: Failed to lock node from params.nh");
    }
    shared_node_ = shared_node;
}
BT::PortsList FibonacciActionNode::providedPorts()
{
    // Return the basic ports provided by the base class, along with a new input port "order"
    return RosActionNode::providedBasicPorts({BT::InputPort<unsigned>("order")});
}

bool FibonacciActionNode::setGoal(RosActionNode::Goal &goal)
{

    // Retrieve the value from the "order" port and set it in the goal object
    getInput("order", goal.order);
    return true; // Return true after successfully setting the goal
}

BT::NodeStatus FibonacciActionNode::onResultReceived(const WrappedResult &wr)
{
    std::stringstream ss;
    ss << "Result received: ";

    // Iterate over the result sequence and log each number received from the server
    for (auto number : wr.result->sequence)
    {
        ss << number << " ";
    }

    // Log the result to ROS
    RCLCPP_INFO(shared_node_->get_logger(), ss.str().c_str());
    return BT::NodeStatus::SUCCESS; // Return success after processing the result
}

BT::NodeStatus FibonacciActionNode::onFailure(BT::ActionNodeErrorCode error)
{
    // Log the error code if the communication fails and return failure
    RCLCPP_ERROR(shared_node_->get_logger(), "Error: %d", error);
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FibonacciActionNode::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    std::stringstream ss;
    ss << "Next number in sequence received: ";

    // Log the partial sequence received as feedback from the action server

    for (auto number : feedback->partial_sequence)
    {
        ss << number << " ";
    }

    // Log the feedback to ROS and return RUNNING
    RCLCPP_INFO(shared_node_->get_logger(), ss.str().c_str());
    return BT::NodeStatus::RUNNING;
}

} // namespace bt_tutorials_ros2_action_client
