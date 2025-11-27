#include "bt_tutorials_ros2/bt_tutorials_ros2_service_client.hpp"

namespace bt_tutorials_ros2_service_client
{

AddTwoIntsNode::AddTwoIntsNode(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
    : RosServiceNode<AddTwoInts>(name, conf, params)
{
    // Convert weak_ptr (from ROS params) to shared_ptr for usage in ROS-related tasks like logging
    auto shared_node = params.nh.lock();
    if (!shared_node)
    {
        throw std::runtime_error("AddTwoIntsNode: Failed to lock node from params.nh");
    }
    shared_node_ = shared_node;
}

BT::PortsList AddTwoIntsNode::providedPorts()
{
    // Return the basic ports provided by the base class, along with new input ports "A" and "B"
    return RosServiceNode::providedBasicPorts({BT::InputPort<unsigned>("A"), BT::InputPort<unsigned>("B")});
}

bool AddTwoIntsNode::setRequest(Request::SharedPtr &request)
{
    // Get numbers to add from the input ports
    getInput("A", request->a);
    getInput("B", request->b);

    // Return true if we were able to set the request correctly
    return true;
}

BT::NodeStatus AddTwoIntsNode::onResponseReceived(const Response::SharedPtr &response)
{
    // Log the sum received from the service server
    RCLCPP_INFO(shared_node_->get_logger(), "Sum: %ld", response->sum);

    // Return success after processing the response
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AddTwoIntsNode::onFailure(BT::ServiceNodeErrorCode error)
{
    // Log the error code if communication with the service server fails and return failure
    RCLCPP_ERROR(shared_node_->get_logger(), "Error: %d", error);

    return BT::NodeStatus::FAILURE;
}

} // namespace bt_tutorials_ros2_service_client
