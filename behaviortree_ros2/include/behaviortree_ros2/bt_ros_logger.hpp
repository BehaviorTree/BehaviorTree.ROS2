#ifndef BT_ROS_LOGGER_H
#define BT_ROS_LOGGER_H

#include <cstring>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include <rclcpp/logging.hpp>
#include "behaviortree_cpp/loggers/abstract_logger.h"

namespace BT
{
/**
 * @brief RosLogger is a very simple logger that
 * displays all the transitions on the console.
 */

class RosLogger : public StatusChangeLogger
{
public:
  RosLogger(const BT::Tree& tree, std::shared_ptr<rclcpp::Node> node);
  ~RosLogger() override;

  virtual void flush() override;

private:
  virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                        NodeStatus status) override;
  std::weak_ptr<rclcpp::Node> node_;
};
}  // namespace BT

#endif  // BT_ROS_LOGGER_H
