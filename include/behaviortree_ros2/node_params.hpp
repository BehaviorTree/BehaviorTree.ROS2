#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>
#include <memory>

namespace BT
{

struct NodeParams
{
  std::shared_ptr<rclcpp::Node> nh;
  std::string server_name;
  std::chrono::milliseconds server_timeout = std::chrono::milliseconds(1000);
};

} // namespace BT
