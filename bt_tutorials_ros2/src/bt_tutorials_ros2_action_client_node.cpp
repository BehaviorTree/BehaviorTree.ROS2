#include "bt_tutorials_ros2/bt_tutorials_ros2_action_client.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

using namespace bt_tutorials_ros2_action_client;

// Define the XML representation of the behavior tree
static const char *xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <Script code=" fib_order:=10 " />
            <FibonacciActionNode order="{fib_order}" />
        </Sequence>
    </BehaviorTree>
</root>
)";

int main(int argc, char **argv)
{
    // Initialize ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a ROS node for the action client
    auto node = std::make_shared<rclcpp::Node>("bt_tutorials_ros2_action_client_node");

    // Params for the Fibonacci Action Node
    BT::RosNodeParams params;
    params.nh = std::weak_ptr<rclcpp::Node>(node); // Note nh is weak_ptr type
    params.default_port_value = "/fibonacci";      // Set the action server name

    // Register the FibonacciActionNode type with the behavior tree factory
    BT::BehaviorTreeFactory factory;
    try
    {
        factory.registerNodeType<FibonacciActionNode>("FibonacciActionNode", params);
    }
    catch (const std::exception &e)
    {
        // Catch and log any errors during node registration
        RCLCPP_ERROR(node->get_logger(), "Error during node registration: %s", e.what());
    }

    // Create the behavior tree from the XML string and tick it
    RCLCPP_INFO(node->get_logger(),
                "Ticking the tree. Ensure %s server from the official ROS tutorials is up and running.",
                params.default_port_value.c_str());
    auto tree = factory.createTreeFromText(xml_text);
    tree.tickWhileRunning();

    // Shutdown ROS 2 client library when finished
    rclcpp::shutdown();
    return 0;
}
