// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_
#define BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_conversions.hpp"

namespace BT
{

/**
 * @brief Abstract class representing a service based BT node
 * @tparam ServiceT Type of service
 */
template<class ServiceT>
class BtServiceNode : public ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::BtServiceNode constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  BtServiceNode(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const NodeConfig& conf)
  : ActionNodeBase(xml_tag_name, conf), service_name_(service_name)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Get the required items from the blackboard
    bt_loop_duration_ =
      config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

    // Now that we have node_ to use, create the service client for this BT service
    std::string remapped_service_name;
    if(getInput("service_name", remapped_service_name))
    {
      service_name_ = remapped_service_name;
    }
    std::string node_namespace;
    node_namespace = node_->get_namespace();
    // Append namespace to the action name
    if(node_namespace != "/") {
      service_name_ = node_namespace + "/" + service_name_;
    }
    service_client_ = node_->create_client<ServiceT>(
      service_name_,
      rmw_qos_profile_services_default,
      callback_group_);

    // Make a request for the service without parameter
    request_ = std::make_shared<typename ServiceT::Request>();

    // Make sure the server is actually there before continuing
    RCLCPP_DEBUG(
      node_->get_logger(), "Waiting for \"%s\" service",
      service_name_.c_str());
    service_client_->wait_for_service();

    RCLCPP_DEBUG(
      node_->get_logger(), "\"%s\" BtServiceNode initialized",
      service_name_.c_str());
  }

  BtServiceNode() = delete;

  virtual ~BtServiceNode()
  {
  }

  /**
   * @brief Any subclass of BtServiceNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = {
      InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
      InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  /**
   * @brief The main override required by a BT service
   * @return NodeStatus Status of tick execution
   */
  NodeStatus tick() override
  {
    if (!request_sent_) {
      on_tick();
      future_result_ = service_client_->async_send_request(request_).share();
      sent_time_ = node_->now();
      request_sent_ = true;
    }
    return check_future();
  }

  /**
   * @brief The other (optional) override required by a BT service.
   */
  void halt() override
  {
    request_sent_ = false;
    resetStatus();
  }

  /**
   * @brief Function to perform some user-defined operation on tick
   * Fill in service request with information if necessary
   */
  virtual void on_tick()
  {
  }

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the service. Could put a value on the blackboard.
   * @param response can be used to get the result of the service call in the BT Node.
   * @return NodeStatus Returns SUCCESS by default, user may override to return another value
   */
  virtual NodeStatus on_completion(std::shared_ptr<typename ServiceT::Response>/*response*/)
  {
    return NodeStatus::SUCCESS;
  }

  /**
   * @brief Check the future and decide the status of BT
   * @return NodeStatus SUCCESS if future complete before timeout, FAILURE otherwise
   */
  virtual NodeStatus check_future()
  {
    auto elapsed = (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
    auto remaining = server_timeout_ - elapsed;

    if (remaining > std::chrono::milliseconds(0)) {
      auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;

      rclcpp::FutureReturnCode rc;
      rc = callback_group_executor_.spin_until_future_complete(future_result_, server_timeout_);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        request_sent_ = false;
        NodeStatus status = on_completion(future_result_.get());
        return status;
      }

      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        on_wait_for_result();
        elapsed = (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
        if (elapsed < server_timeout_) {
          return NodeStatus::RUNNING;
        }
      }
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "Node timed out while executing service call to %s.", service_name_.c_str());
    request_sent_ = false;
    return NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation after a timeout waiting
   * for a result that hasn't been received yet
   */
  virtual void on_wait_for_result()
  {
  }

protected:
  /**
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   */
  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  std::string service_name_;
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;
  std::shared_ptr<typename ServiceT::Request> request_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds server_timeout_;

  // The timeout value for BT loop execution
  std::chrono::milliseconds bt_loop_duration_;

  // To track the server response when a new request is sent
  std::shared_future<typename ServiceT::Response::SharedPtr> future_result_;
  bool request_sent_{false};
  rclcpp::Time sent_time_;
};

/// Method to register the bt action into a factory.
template <class DerivedT> static
  void register_bt_action(BehaviorTreeFactory& factory,
    const std::string& registration_ID,
    const std::string& service_name)
{
  NodeBuilder builder = [=](const std::string& name, const NodeConfig& config)
  {
    return std::make_unique<DerivedT>(name, service_name, config);
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = DerivedT::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());
  factory.registerBuilder(manifest, builder);
}

}  // namespace BT

#endif  // BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_
