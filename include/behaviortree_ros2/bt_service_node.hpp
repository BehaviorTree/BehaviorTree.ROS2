// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2020 Davide Faconti
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

#ifndef BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_
#define BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "rclcpp/rclcpp.hpp"

namespace BT
{

/**
 * Base Action to implement a ROS Service
 */
template<class ServiceT>
class RosServiceNode : public BT::SyncActionNode
{
protected:

  RosServiceNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
   BT::SyncActionNode(name, conf), node_(nh) { }

public:

  using BaseClass    = RosServiceNode<ServiceT>;
  using ServiceType  = ServiceT;
  using RequestType  = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  RosServiceNode() = delete;

  virtual ~RosServiceNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosAction<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("service_name", "name of the ROS service"),
      InputPort<unsigned>("timeout", 100, "timeout to connect to server (milliseconds)")
      };
  }

  /// User must implement this method.
  virtual void sendRequest(RequestType& request) = 0;

  /// Method (to be implemented by the user) to receive the reply.
  /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
  virtual NodeStatus onResponse( const ResponseType& rep) = 0;

  enum FailureCause{
    MISSING_SERVER = 0,
    FAILED_CALL = 1
  };

  /// Called when a service call failed. Can be overriden by the user.
  virtual NodeStatus onFailedRequest(FailureCause failure)
  {
    return NodeStatus::FAILURE;
  }

protected:

  ros::ServiceClient service_client_;

  typename ServiceT::Response reply_;

  // The node that will be used for any ROS operations
  ros::NodeHandle& node_;

  BT::NodeStatus tick() override
  {
    if( !service_client_.isValid() ){
      std::string server = getInput<std::string>("service_name").value();
      service_client_ = node_.serviceClient<ServiceT>( server );
    }

    unsigned msec;
    getInput("timeout", msec);
    ros::Duration timeout(static_cast<double>(msec) * 1e-3);

    bool connected = service_client_.waitForExistence(timeout);
    if( !connected ){
      return onFailedRequest(MISSING_SERVER);
    }

    typename ServiceT::Request request;
    sendRequest(request);
    bool received = service_client_.call( request, reply_ );
    if( !received )
    {
      return onFailedRequest(FAILED_CALL);
    }
    return onResponse(reply_);
  }
};


/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterRosService(BT::BehaviorTreeFactory& factory,
                     const std::string& registration_ID,
                     ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = RosServiceNode< typename DerivedT::ServiceType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_