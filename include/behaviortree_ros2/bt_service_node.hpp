#ifndef BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_
#define BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
namespace BT
{

  struct ServiceNodeParams
  {
    std::shared_ptr<rclcpp::Node> nh;
    std::string service_name;
    std::chrono::milliseconds service_timeout;
  };

  enum ServiceNodeErrorCode
  {
    MISSING_SERVER = 0,
    FAILED_CALL = 1
  };

  /**
   * @brief Abstract class representing use to call a ROS2 Service (client).
   *
   * It will try to be non-blocking for the entire duration of the call.
   * The derived class whould reimplement the virtual methods as described below.
   */
  template <class ServiceT>
  class RosServiceNode : public BT::ActionNodeBase
  {

  public:
    // Type definitions
    using ServiceType = ServiceT;
    using ServiceClient = typename rclcpp::Client<ServiceT>;
    using RequestType = typename ServiceT::Request;
    using ResponseType = typename ServiceT::Response::SharedPtr;
    using Params = ServiceNodeParams;

    /** You are not supposed to instantiate this class directly, the factory will do it.
     * To register this class into the factory, use:
     *
     *    RegisterRosService<DerivedClasss>(factory, params)
     *
     * */
    explicit RosServiceNode(const std::string &instance_name,
                            const BT::NodeConfiguration &conf,
                            const ServiceNodeParams &params,
                            typename std::shared_ptr<ServiceClient> external_service_client = {});

    virtual ~RosServiceNode() = default;

    NodeStatus tick() override final;

    void halt() override;


    /// User must implement this method.
    virtual void sendRequest(typename ServiceType::Request::SharedPtr request) = 0;

    /// Method (to be implemented by the user) to receive the reply.
    /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
    virtual BT::NodeStatus onResponse(const ResponseType &rep) = 0;

    /// Called when a service call failed. Can be overriden by the user.
    virtual BT::NodeStatus onFailedRequest(ServiceNodeErrorCode failure)
    {
      return NodeStatus::FAILURE;
    }

  protected:
    std::shared_ptr<rclcpp::Node> node_;
    const std::string service_name_;
    const std::chrono::milliseconds service_timeout_;

  private:

    typename std::shared_ptr<ServiceClient> service_client_;
    std::shared_future<typename ServiceT::Response::SharedPtr> _future_response;
    BT::NodeStatus _result;
  };

  /// Method to register the service into a factory.
  /// It gives you the opportunity to set the ros::NodeHandle.
  template <class DerivedT>
  static void RegisterRosService(BT::BehaviorTreeFactory &factory,
                                 const std::string &registration_ID,
                                 const ServiceNodeParams &params,
                                 std::shared_ptr<typename DerivedT::ServiceClient> external_client = {})
  {
    NodeBuilder builder = [=](const std::string &name, const NodeConfiguration &config)
    {
      return std::make_unique<DerivedT>(name, config, params, external_client);
    };

    TreeNodeManifest manifest;
    manifest.type = getType<DerivedT>();
    manifest.ports = DerivedT::providedPorts();
    manifest.registration_ID = registration_ID;
    const auto &basic_ports = DerivedT::providedPorts();
    manifest.ports.insert(basic_ports.begin(), basic_ports.end());
    factory.registerBuilder(manifest, builder);
  }

  //----------------------------------------------------------------
  //---------------------- DEFINITIONS -----------------------------
  //----------------------------------------------------------------

  template <class T>
  inline RosServiceNode<T>::RosServiceNode(const std::string &instance_name,
                                           const BT::NodeConfiguration &conf,
                                           const ServiceNodeParams &params,
                                           typename std::shared_ptr<ServiceClient> external_service_client) : BT::ActionNodeBase(instance_name, conf),
                                                                                                              node_(params.nh),
                                                                                                              service_name_(params.service_name),
                                                                                                              service_timeout_(params.service_timeout)
  {
    if (external_service_client)
    {
      service_client_ = external_service_client;
    }
    else
    {
      service_client_ = node_->create_client<T>(service_name_);
      RCLCPP_INFO_STREAM(
          node_->get_logger(), "Service client created for " << service_name_ << " with timeout " << service_timeout_.count() << "ms");
    }

    bool found = service_client_->wait_for_service(service_timeout_);
    if (!found)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Service [%s] is not reachable. This will be checked only once", service_name_.c_str());
    }
  }


template <class T>
inline NodeStatus RosServiceNode<T>::tick()
{

  if (this->status() == BT::NodeStatus::IDLE)
  {
    while (!service_client_->wait_for_service(service_timeout_))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(node_->get_logger(), "Client interrupted while waiting for service to appear.");
        return BT::NodeStatus::FAILURE;
      }
      // RCLCPP_INFO(_node->get_logger(), "waiting for service to appear...");
    
    }

    RequestType req;

    typename ServiceType::Request::SharedPtr request = std::make_shared<RequestType>(req) ;    

    sendRequest(request);
    

    auto response_received_callback = [this](typename rclcpp::Client<ServiceType>::SharedFuture future)
    {
      auto result = future.get();
      _result = onResponse(result);
    };
    _future_response = service_client_->async_send_request(request, response_received_callback);
    _result = BT::NodeStatus::RUNNING;
  }
  rclcpp::spin_some(node_);
  return _result;
}

template<class T> inline
  void RosServiceNode<T>::halt()
{
  if( status() == NodeStatus::RUNNING )
  {
    RCLCPP_ERROR( node_->get_logger(),
                 "Halted");
  }
}

} // namespace BT

#endif // BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_