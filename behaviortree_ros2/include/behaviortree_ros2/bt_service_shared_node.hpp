// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2023 Davide Faconti
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

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/bt_factory.h"

#include "behaviortree_ros2/ros_node_params.hpp"
#include "behaviortree_ros2/bt_service_error_code.hpp"

namespace BT
{
/**
 * @brief Abstract class use to wrap rclcpp::Client<>
 *
 * For instance, given the type AddTwoInts described in this tutorial:
 * https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html
 *
 * the corresponding wrapper would be:
 *
 * class AddTwoNumbers: public RosServiceSharedNode<example_interfaces::srv::AddTwoInts>
 *
 * RosServiceSharedNode will try to be non-blocking for the entire duration of the call.
 * The derived class must reimplement the virtual methods as described below.
 *
 * The name of the service will be determined as follows:
 *
 * 1. If a value is passes in the InputPort "service_name", use that
 * 2. Otherwise, use the value in RosNodeParams::default_port_value
 */
template<class ServiceT, class Derived>
class RosServiceSharedNode : public BT::ActionNodeBase
{

public:
  // Type definitions
  using ServiceClient = typename rclcpp::Client<ServiceT>;
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;

  /** To register this class into the factory, use:
   *
   *    factory.registerNodeType<>(node_name, params);
   */
  explicit RosServiceSharedNode(const std::string & instance_name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params);

  virtual ~RosServiceSharedNode() = default;

  /**
   * @brief Any subclass of RosServiceSharedNode that has ports must implement a
   * providedPorts method and call providedBasicPorts in it.
   *
   * @param addition Additional ports to add to BT port list
   * @return PortsList containing basic ports along with node-specific ports
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = {
      InputPort<std::string>("service_name", "__default__placeholder__", "Service name")
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

  NodeStatus tick() override final;

  /// The default halt() implementation.
  void halt() override;

  /** setRequest is a callback that allows the user to set
   * the request message (ServiceT::Request).
   *
   * @param request  the request to be sent to the service provider.
   *
   * @return false if the request should not be sent. In that case,
   * RosServiceSharedNode::onFailure(INVALID_REQUEST) will be called.
   */
  virtual bool setRequest(typename Request::SharedPtr& request) = 0;

  /** Callback invoked when the response is received by the server.
   * It is up to the user to define if this returns SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) = 0;

  /** Callback invoked when something goes wrong; you can override it.
   * It must return either SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFailure(ServiceNodeErrorCode /*error*/)
  {
    return NodeStatus::FAILURE;
  }

protected:

  std::shared_ptr<rclcpp::Node> node_;
  std::string prev_service_name_;
  bool service_name_may_change_ = false;
  const std::chrono::milliseconds service_timeout_;
  const std::chrono::milliseconds wait_for_service_timeout_;
  static typename std::shared_ptr<ServiceClient> service_client_;
  static bool shared_resource_initialized;
private:
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> callback_group_executor_;
  static std::mutex mutex_;

  std::shared_future<typename Response::SharedPtr> future_response_;

  rclcpp::Time time_request_sent_;
  NodeStatus on_feedback_state_change_;
  bool response_received_;
  typename Response::SharedPtr response_;

  bool createClient(const std::string &service_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template<class T, class D> inline
  RosServiceSharedNode<T, D>::RosServiceSharedNode(const std::string & instance_name,
                                    const NodeConfig &conf,
                                    const RosNodeParams& params):
  BT::ActionNodeBase(instance_name, conf),
  node_(params.nh),
  service_timeout_(params.server_timeout),
  wait_for_service_timeout_(params.wait_for_server_timeout)
{
  // check port remapping
  auto portIt = config().input_ports.find("service_name");
  if(portIt != config().input_ports.end())
  {
    const std::string& bb_service_name = portIt->second;

    if(bb_service_name.empty() || bb_service_name == "__default__placeholder__")
    {
      if(params.default_port_value.empty()) {
        throw std::logic_error(
          "Both [service_name] in the InputPort and the RosNodeParams are empty.");
      }
      else {
        createClient(params.default_port_value);
      }
    }
    else if(!isBlackboardPointer(bb_service_name))
    {
      // If the content of the port "service_name" is not
      // a pointer to the blackboard, but a static string, we can
      // create the client in the constructor.
      createClient(bb_service_name);
    }
    else {
      service_name_may_change_ = true;
      // createClient will be invoked in the first tick().
    }
  }
  else {

    if(params.default_port_value.empty()) {
      throw std::logic_error(
        "Both [service_name] in the InputPort and the RosNodeParams are empty.");
    }
    else {
      createClient(params.default_port_value);
    }
  }
}

template<class T, class D> inline
  bool RosServiceSharedNode<T, D>::createClient(const std::string& service_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(service_name.empty())
  {
    throw RuntimeError("service_name is empty");
  }

  if (!shared_resource_initialized) {
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    if (callback_group_executor_ == nullptr) {
      callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
    callback_group_executor_->add_callback_group(callback_group_, node_->get_node_base_interface());
    service_client_ = node_->create_client<T>(service_name, rmw_qos_profile_services_default, callback_group_);
    prev_service_name_ = service_name;
    shared_resource_initialized = true;
  }


  bool found = service_client_->wait_for_service(wait_for_service_timeout_);
  if(!found)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s: Service with name '%s' is not reachable.",
                 name().c_str(), prev_service_name_.c_str());
  }
  return found;
}

template<class T, class D> inline
  NodeStatus RosServiceSharedNode<T, D>::tick()
{
  // First, check if the service_client_ is valid and that the name of the
  // service_name in the port didn't change.
  // otherwise, create a new client
  if(!service_client_ || (status() == NodeStatus::IDLE && service_name_may_change_))
  {
    std::string service_name;
    getInput("service_name", service_name);
    if(prev_service_name_ != service_name)
    {
      createClient(service_name);
    }
  }

  auto CheckStatus = [](NodeStatus status)
  {
    if( !isStatusCompleted(status) )
    {
      throw std::logic_error("RosServiceSharedNode: the callback must return either SUCCESS or FAILURE");
    }
    return status;
  };

  // first step to be done only at the beginning of the Action
  if (status() == BT::NodeStatus::IDLE)
  {
    setStatus(NodeStatus::RUNNING);

    response_received_ = false;
    future_response_ = {};
    on_feedback_state_change_ = NodeStatus::RUNNING;
    response_ = {};

    typename Request::SharedPtr request = std::make_shared<Request>();

    if( !setRequest(request) )
    {
      return CheckStatus( onFailure(INVALID_REQUEST) );
    }

    future_response_ = service_client_->async_send_request(request).share();
    time_request_sent_ = node_->now();

    return NodeStatus::RUNNING;
  }

  if (status() == NodeStatus::RUNNING)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_group_executor_->spin_some();

    // FIRST case: check if the goal request has a timeout
    if( !response_received_ )
    {
      auto const nodelay = std::chrono::milliseconds(0);
      auto const timeout = rclcpp::Duration::from_seconds( double(service_timeout_.count()) / 1000);

      auto ret = callback_group_executor_->spin_until_future_complete(future_response_, nodelay);

      if (ret != rclcpp::FutureReturnCode::SUCCESS)
      {
        if( (node_->now() - time_request_sent_) > timeout )
        {
          return CheckStatus( onFailure(SERVICE_TIMEOUT) );
        }
        else{
          return NodeStatus::RUNNING;
        }
      }
      else
      {
        response_received_ = true;
        response_ = future_response_.get();
        future_response_ = {};

        if (!response_) {
          throw std::runtime_error("Request was rejected by the service");
        }
      }
    }

    // SECOND case: response received
    return CheckStatus( onResponseReceived( response_ ) );
  }
  return NodeStatus::RUNNING;
}

template<class T, class D> inline
  void RosServiceSharedNode<T, D>::halt()
{
  if( status() == NodeStatus::RUNNING )
  {
    resetStatus();
  }
}

template <class TopicT, class Derived>
bool RosServiceSharedNode<TopicT, Derived>::shared_resource_initialized = false;

template <class TopicT, class Derived>
typename std::shared_ptr<rclcpp::Client<TopicT>> RosServiceSharedNode<TopicT, Derived>::service_client_ = nullptr;

template <class ActionT, class Derived>
std::mutex RosServiceSharedNode<ActionT, Derived>::mutex_;

template <class ActionT, class Derived>
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> RosServiceSharedNode<ActionT,Derived>::callback_group_executor_ = nullptr;
}  // namespace BT
