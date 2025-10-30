#include "behaviortree_ros2/bt_ros_logger.hpp"

namespace BT
{

RosLogger::RosLogger(const BT::Tree& tree, std::shared_ptr<rclcpp::Node> node) : StatusChangeLogger(tree.rootNode()), node_(node)
{}
RosLogger::~RosLogger()
{}

void RosLogger::callback(Duration timestamp, const TreeNode& node,
                             NodeStatus prev_status, NodeStatus status)
{
  using namespace std::chrono;

  // get ros node pointer
  auto ros_node = node_.lock();

  if (ros_node) {

    constexpr const char* whitespaces = "                         ";
    constexpr const size_t ws_count = 25;

    double since_epoch = duration<double>(timestamp).count();

    RCLCPP_DEBUG(
      ros_node->get_logger(), "[%.3f]: %s%s %s -> %s",
      since_epoch, node.name().c_str(),
          &whitespaces[std::min(ws_count, node.name().size())],
          toStr(prev_status, true).c_str(), toStr(status, true).c_str());
  }
}

void RosLogger::flush()
{
}

}  // namespace BT