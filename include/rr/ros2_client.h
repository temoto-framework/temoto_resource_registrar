#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_CLIENT_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_CLIENT_H

#include "rclcpp/rclcpp.hpp"

#include "temoto_resource_registrar/rr_client_base.h"

#include "rr/message_serializer.h"
#include "rr/ros2_query.h"
#include <functional>

/**
 * @brief A wrapper class for the temoto_resource_registrar::RRClientBase. Provides templating to support multiple message types.
 * 
 * @tparam ServiceClass - type of message this client supports
 */
template <class ServiceClass>
class Ros2Client : public temoto_resource_registrar::RrClientBase
{
public:
  typedef std::function<void(const std::shared_ptr<typename ServiceClass::Request> &,
                             std::shared_ptr<typename ServiceClass::Response>,
                             const temoto_resource_registrar::Status &)>
      UserStatusCb;

  Ros2Client(const std::string &rr, const std::string &name) : temoto_resource_registrar::RrClientBase(rr, name)
  {
    //ROS_INFO_STREAM("constructing Ros2Client " << rr << " id: " << name);
    //client_ = nh_.serviceClient<ServiceClass>(id());

    node_ = rclcpp::Node::make_shared(name);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("ros2_client.h"), "constructing Ros2Client: " << id());
    client_ = node_->create_client<ServiceClass>(id());
  }

  void setNode(std::shared_ptr<rclcpp::Node> node)
  {
    node_ = node;
  }

  void initialize()
  {
    client_ = node_->create_client<ServiceClass>(id());
  }

  /**
 * @brief Invokes a call to the Ros2Server responsible for serving requests of that type. 
 * The request elements are modified to provide a response
 * 
 * @param request - A user defined ROS2 srv type request.
 */
  void invoke(ServiceClass &request)
  {

   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "invoke ServiceClass");
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }


    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "invoke request in async for server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), id().c_str());

    auto result = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OK");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "NOK");
    }
  }

  /**
 * @brief Invokes a call to the Ros2Server responsible for serving requests of that type. 
 * The request elements are modified to provide a response
 * 
 * @param wrapped_request - A user defined Ros2Query type request
 */
  void invoke(Ros2Query<ServiceClass> &wrapped_request)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "invoke wrapped_request");
    while (!client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "invoke request in async for server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), id().c_str());

    auto result = client_->async_send_request(wrapped_request.request());

    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OK");

      wrapped_request.response(result.get());
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "NOK");
    }
  }

  void registerUserStatusCb(const std::string &request_id, const UserStatusCb &user_status_cb)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ("registerUserStatusCb - " + id() + " request: " + request_id).c_str());
    status_callbacks_[request_id] = user_status_cb;

    //ROS_INFO_STREAM("check to see if it really registered: " << hasRegisteredCb(request_id) << " nr of callbacks: " << status_callbacks_.size());

    status_query_ids_.push_back(request_id);
  }

  void internalStatusCallback(const std::string &request_id, const temoto_resource_registrar::Status &status)
  {

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ("internalStatusCallback - " + id() + " request: " + request_id).c_str());

    //ROS_INFO_STREAM("Determinging if client cas callback for id " << request_id << " nr of callbacks: " << status_callbacks_.size());
    if (hasRegisteredCb(request_id))
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "req size: %i", status.serialised_request_.size());
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "res size: %i", status.serialised_response_.size());

      std::shared_ptr<typename ServiceClass::Request> request = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Request>(status.serialised_request_);
      std::shared_ptr<typename ServiceClass::Response> response = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(status.serialised_response_);

      //ServiceClass query;
      //query.request = request;
      //query.response = response;

      //query->request = request;

      response->temoto_metadata.request_id = status.id_;

      status_callbacks_[request_id](request, response, status);

      //user_status_cb_(query, status); // TODO: needs exception handling
    }
  }

  bool hasRegisteredCb(const std::string &request_id) const
  {
    //ROS_INFO_STREAM("hasRegisteredCb " << " - " << id() << " request: " << request_id);
    return status_callbacks_.count(request_id) > 0;
  }

protected:
private:
  typename rclcpp::Client<ServiceClass>::SharedPtr client_;

  std::shared_ptr<rclcpp::Node> node_;

  //UserStatusCb user_status_cb_;
  std::unordered_map<std::string, UserStatusCb> status_callbacks_;
};

#endif