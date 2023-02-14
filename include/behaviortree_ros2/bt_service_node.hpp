#ifndef BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_
#define BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_

#include <memory>
#include <string>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/node_params.hpp"

namespace BT
{

enum ServiceNodeErrorCode
{
  SERVICE_UNREACHABLE,
  SERVICE_TIMEOUT,
  INVALID_REQUEST,
  SERVICE_ABORTED
};

/**
 * @brief Abstract class representing use to call a ROS2 Service (client).
 *
 * It will try to be non-blocking for the entire duration of the call.
 * The derived class whould reimplement the virtual methods as described below.
 */
template<class ServiceT>
class RosServiceNode : public BT::ActionNodeBase
{

public:
  // Type definitions
  using ServiceClient = typename rclcpp::Client<ServiceT>;
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;

  /** You are not supposed to instantiate this class directly, the factory will do it.
   * To register this class into the factory, use:
   *
   *    RegisterRosAction<DerivedClasss>(factory, params)
   *
   * Note that if the external_action_client is not set, the constructor will build its own.
   * */
  explicit RosServiceNode(const std::string & instance_name,
                         const BT::NodeConfig& conf,
                         const NodeParams& params,
                         typename std::shared_ptr<ServiceClient> external_service_client = {});

  virtual ~RosServiceNode() = default;

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = {
      InputPort<std::string>("service_name", "Service name")
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

  /** setRequest is a callback invoked to return the request message (ServiceT::Request).
   * If conditions are not met, it should return "false" and the BT::Action
   * will return FAILURE.
   */
  virtual bool setRequest(typename Request::SharedPtr& request) = 0;

  /** Callback invoked when the response is received by the server.
   * It is up to the user to define if the BT::action returns SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) = 0;

  /** Callback invoked when something goes wrong.
   * It must return either SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFailure(ServiceNodeErrorCode error)
  { 
    (void) error;
    return NodeStatus::FAILURE;
  }

protected:

  std::shared_ptr<rclcpp::Node> node_;
  std::string prev_service_name_;
  bool service_name_may_change_ = false;
  const std::chrono::milliseconds service_timeout_;

private:

  bool createClient(const std::string &service_name);

  typename std::shared_ptr<ServiceClient> service_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::shared_future<typename Response::SharedPtr> future_response_;

  rclcpp::Time time_request_sent_;
  NodeStatus on_feedback_state_change_;
  bool response_received_;
  typename Response::SharedPtr response_;
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template<class T> inline
  RosServiceNode<T>::RosServiceNode(const std::string & instance_name,
                                  const NodeConfig &conf,
                                  const NodeParams& params,
                                  typename std::shared_ptr<ServiceClient> external_service_client):
  BT::ActionNodeBase(instance_name, conf),
  node_(params.nh),
  service_timeout_(params.server_timeout)
{
  if( external_service_client )
  {
    service_client_ = external_service_client;
  }
  else
  {
    // If the content of the port "action_name" is not
    // a pointer to the blackboard, but a static string, we can
    // create the client in the constructor.
    const std::string service_name = config().input_ports.at("service_name");    
    if(service_name.empty())
    {
      if(params.default_server_name.empty())
      {
        throw RuntimeError("Both default_server_name  and service_name is empty");
      }
      createClient(params.default_server_name);
    }
    else if(!isBlackboardPointer(service_name))
    {
      createClient(service_name);
    }
    else {
      service_name_may_change_ = true;
    }


    // std::string remapped_service_name;
    // if (getInput("server_name", remapped_service_name)) {
    //   service_name_ = remapped_service_name;
    // }
    // std::string node_namespace = node_->get_namespace();
    // // Append namespace to the service name
    // if(node_namespace != "/" && service_name_.front() != '/') {
    //   service_name_ = node_namespace + "/" + service_name_;
    // }
    // callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // 
    // service_client_ = node_->create_client<T>(service_name_, qos_profile, callback_group_);

    // bool found = service_client_->wait_for_service(service_timeout_);
    // if(!found)
    // {
    //   RCLCPP_ERROR(node_->get_logger(),
    //                "Service [%s] is not reachable. This will be checked only once", service_name_.c_str());
    // }
  }
}

template<class T> inline
  bool RosServiceNode<T>::createClient(const std::string& service_name)
{
  if(service_name.empty())
  {
    throw RuntimeError("service_name is empty");
  }
  
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const rmw_qos_profile_t& qos_profile{rmw_qos_profile_services_default};
  service_client_ = node_->create_client<T>(service_name, qos_profile, callback_group_);
  prev_service_name_ = service_name;

  bool found = service_client_->wait_for_service(service_timeout_);
  if(!found)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s: Service with name '%s' is not reachable.",
                 name().c_str(), prev_service_name_.c_str());
  }
  return found;
}

template<class T> inline
  NodeStatus RosServiceNode<T>::tick()
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
    if( status != NodeStatus::SUCCESS && status != NodeStatus::FAILURE )
    {
      throw std::logic_error("RosServiceNode: the callback must return either SUCCESS or FAILURE");
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
    rclcpp::spin_some(node_);

    // FIRST case: check if the goal request has a timeout
    if( !response_received_ )
    {
      auto nodelay = std::chrono::milliseconds(0);
      auto timeout = rclcpp::Duration::from_seconds( double(service_timeout_.count()) / 1000);

      if (rclcpp::spin_until_future_complete(node_, future_response_, nodelay) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_WARN_ONCE( node_->get_logger(), "waiting response confirmation" );
        if( (node_->now() - time_request_sent_) > timeout )
        {
          RCLCPP_WARN( node_->get_logger(), "TIMEOUT" );
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

template<class T> inline
  void RosServiceNode<T>::halt()
{
  if( status() == NodeStatus::RUNNING )
  {
    resetStatus();
  }
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_
