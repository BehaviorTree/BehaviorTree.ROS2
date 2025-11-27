/**
 * @file bt_tutorials_ros2_service_client.hpp
 * @brief Header file for the AddTwoIntsNode, a custom BehaviorTree node for interacting with the official ROS 2
 * "/add_two_ints" service.
 *
 * @author Janak Panthi (Crasun Jans)
 *
 * @details Implements the AddTwoInts service client tutorial from the "Integration with ROS2" section on the
 * official BehaviorTree.CPP website. This node interacts with a ROS 2 service server to send two integers as a
 * request and processes the resulting sum.
 *
 * @license MIT
 */

#ifndef BT_TUTORIALS_ROS2_SERVICE_CLIENT_HPP
#define BT_TUTORIALS_ROS2_SERVICE_CLIENT_HPP

#include <behaviortree_cpp/action_node.h>          // Base class for action nodes in BehaviorTreeCpp.
#include <behaviortree_ros2/bt_service_node.hpp>   // BT wrapper for ROS2 service nodes.
#include <example_interfaces/srv/add_two_ints.hpp> // Service definition for AddTwoInts service.
#include <string>                                  // For handling string types.

namespace bt_tutorials_ros2_service_client
{
// Alias for the AddTwoInts service type.
using AddTwoInts = example_interfaces::srv::AddTwoInts;

/**
 * @brief Class representing a custom service node for the AddTwoInts service in a BehaviorTree.
 *
 * This class derives from the `RosServiceNode` provided by the `behaviortree_ros2` library.
 * It implements all necessary callbacks and methods to interact with a service server
 * to perform the AddTwoInts service call.
 */
class AddTwoIntsNode : public BT::RosServiceNode<AddTwoInts>
{
  public:
    /**
     * @brief Constructor for the AddTwoIntsNode.
     *
     * Initializes the service node with the given name, configuration, and ROS node parameters.
     * This allows the service node to communicate with the ROS 2 system and the service server.
     *
     * @param name The name of the node in the behavior tree.
     * @param conf The configuration parameters for the behavior tree node.
     * @param params ROS node parameters used to configure the service node.
     */
    AddTwoIntsNode(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params);

    /**
     * @brief Provides the ports required by this service node.
     *
     * This static function defines the input and output ports for the node, merging the
     * base class ports with any additional ones specific to this derived class.
     *
     * @return A list of ports this service node provides.
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Sends a request to the service server when the tree node is ticked.
     *
     * This function is called when the behavior tree ticks the node. It sends the request to the
     * service server.
     *
     * @param request The request to be sent to the service server.
     * @return True if the request was successfully set, otherwise false.
     */
    bool setRequest(Request::SharedPtr &request) override;

    /**
     * @brief Callback executed when a response is received from the service server.
     *
     * This function is invoked when the service server sends a response. Based on the response,
     * it will return either `SUCCESS` or `FAILURE` to indicate the outcome of the service call.
     *
     * @param response The response received from the service server.
     * @return The status of the node after processing the response.
     */
    BT::NodeStatus onResponseReceived(const Response::SharedPtr &response) override;

    /**
     * @brief Callback invoked when the service client encounters a communication failure.
     *
     * This callback handles failure cases where the communication between the service client
     * and the server fails. The node status is updated to either `SUCCESS` or `FAILURE` based on
     * the error received.
     *
     * @param error The error code indicating the failure reason.
     * @return The status of the node after handling the failure.
     */
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

  private:
    std::shared_ptr<rclcpp::Node> shared_node_; // Shared pointer to the ROS node for logging and other tasks.
};

} // namespace bt_tutorials_ros2_service_client

#endif // BT_TUTORIALS_ROS2_SERVICE_CLIENT_HPP
