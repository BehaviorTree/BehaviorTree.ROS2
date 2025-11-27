/**
 * @file bt_tutorials_ros2_action_client.hpp
 * @brief Header file for the FibonacciActionNode, a custom BehaviorTree node for interacting with the official ROS 2
 * "/fibonacci" action server.
 *
 * @author Janak Panthi (Crasun Jans)
 *
 * @details Implements the Fibonacci action client tutorial from the "Integration with ROS2" section on the
 * official BehaviorTree.CPP website. This node sends goals to a Fibonacci action server, processes its feedback,
 * and handles the result.
 *
 * @license MIT
 */

#ifndef BT_TUTORIALS_ROS2_ACTION_CLIENT_HPP
#define BT_TUTORIALS_ROS2_ACTION_CLIENT_HPP

#include <action_tutorials_interfaces/action/fibonacci.hpp> // Action definition for Fibonacci action.
#include <behaviortree_cpp/action_node.h>                   // Base class for action nodes in BehaviorTreeCpp.
#include <behaviortree_ros2/bt_action_node.hpp>             // BT wrapper for ROS2 actions.
#include <string>                                           // For handling string types.

namespace bt_tutorials_ros2_action_client
{
// Alias for the Fibonacci action type.
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

/**
 * @brief Class representing a custom action node for the Fibonacci action in a BehaviorTree.
 *
 * This class derives from the `RosActionNode` provided by the `behaviortree_ros2` library.
 * It implements all necessary callbacks and methods to interact with an action server
 * to perform the Fibonacci action.
 */
class FibonacciActionNode : public BT::RosActionNode<Fibonacci>
{
  public:
    // Type alias for the GoalHandle specific to the Fibonacci action.
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    /**
     * @brief Constructor for the FibonacciActionNode.
     *
     * Initializes the action node with the given name, configuration, and ROS node parameters.
     * This allows the action node to interact with the ROS 2 system and communicate with the action server.
     *
     * @param name The name of the node in the behavior tree.
     * @param conf The configuration parameters for the behavior tree node.
     * @param params ROS node parameters used to configure the action node.
     */
    FibonacciActionNode(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params);

    /**
     * @brief Provides the ports required by this action node.
     *
     * This static function defines the input and output ports for the node, merging the
     * base class ports with any additional ones specific to this derived class.
     *
     * @return A list of ports this action node provides.
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Sends a request to the action server when the tree node is ticked.
     *
     * This function is called when the behavior tree ticks the node. It sends the goal to the
     * action server.
     *
     * @param goal The goal to be sent to the action server.
     * @return True if the goal was successfully set, otherwise false.
     */
    bool setGoal(RosActionNode::Goal &goal) override;

    /**
     * @brief Callback executed when a result is received from the action server.
     *
     * This function is invoked when the action server sends a result. Based on the result,
     * it will return either `SUCCESS` or `FAILURE` to indicate the outcome of the action.
     *
     * @param wr The wrapped result received from the action server.
     * @return The status of the node after processing the result.
     */
    BT::NodeStatus onResultReceived(const WrappedResult &wr) override;

    /**
     * @brief Callback invoked when the action client encounters a communication failure.
     *
     * This callback handles failure cases where the communication between the action client
     * and the server fails. The node status is updated to either `SUCCESS` or `FAILURE` based on
     * the error received.
     *
     * @param error The error code indicating the failure reason.
     * @return The status of the node after handling the failure.
     */
    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

    /**
     * @brief Callback for receiving feedback from the action server.
     *
     * This function is invoked when feedback is received from the action server. The feedback
     * can be used to determine the current progress of the action. If necessary, the action can
     * be aborted based on feedback, or the node can return `SUCCESS` or `FAILURE` if the action completes.
     *
     * @param feedback The feedback received from the action server.
     * @return The status of the node after processing the feedback.
     */
    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

  private:
    std::shared_ptr<rclcpp::Node> shared_node_; // shared pointer to the ROS node
};

} // namespace bt_tutorials_ros2_action_client

#endif // BT_TUTORIALS_ROS2_ACTION_CLIENT_HPP
