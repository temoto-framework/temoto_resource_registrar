#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_CLIENT_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_CLIENT_H

#include "ros/ros.h"
#include "rr/message_serializer.h"
#include "rr/ros1_query.h"
#include <functional>

/**
 * @brief A wrapper class for the temoto_resource_registrar::RRClientBase. Provides templating to support multiple message types.
 * 
 * @tparam ServiceClass - type of message this client supports
 */
template <class ServiceClass>
class Ros1Client : public temoto_resource_registrar::RrClientBase
{
public:
  typedef std::function<void(ServiceClass, temoto_resource_registrar::Status)> UserStatusCb;

  Ros1Client(const std::string &rr, const std::string &name) : temoto_resource_registrar::RrClientBase(rr, name)
  {
    TEMOTO_INFO_STREAM_("constructing Ros1Client " << rr << " id: " << name);
    client_ = nh_.serviceClient<ServiceClass>(id());
  }

  /**
 * @brief Invokes a call to the Ros1Server responsible for serving requests of that type. 
 * The request elements are modified to provide a response
 * 
 * @param request - A user defined ROS1 srv type request.
 */
  void invoke(ServiceClass &request)
  {
    TEMOTO_INFO_STREAM_("invoke for ServiceClass started. Targeting " << id());
    if (client_.call(request))
    {
      TEMOTO_INFO_STREAM_("invoke OK " << request.response.temoto_metadata.request_id);
    }
    else
    {
      ROS_INFO("FAIL");
    }
    TEMOTO_INFO_STREAM_("invoke for ServiceClass completed");
  }

  /**
 * @brief Invokes a call to the Ros1Server responsible for serving requests of that type. 
 * The request elements are modified to provide a response
 * 
 * @param wrapped_request - A user defined Ros1Query type request
 */
  void invoke(Ros1Query<ServiceClass> &wrapped_request)
  {
    TEMOTO_INFO_STREAM_("invoke for Ros1Query wrapper started");
    ServiceClass service_call = wrapped_request.rosQuery();

    invoke(service_call);

    wrapped_request = Ros1Query<ServiceClass>(service_call);
    TEMOTO_INFO_STREAM_("invoke for Ros1Query wrapper completed");
  }

  void registerUserStatusCb(const std::string &request_id, const UserStatusCb &user_status_cb)
  {
    TEMOTO_INFO_STREAM_("registerUserStatusCb "
                    << " - " << id() << " request: " << request_id);

    status_callbacks_[request_id] = user_status_cb;

    TEMOTO_INFO_STREAM_("check to see if it really registered: " << hasRegisteredCb(request_id) << " nr of callbacks: " << status_callbacks_.size());

    status_query_ids_.push_back(request_id);
  }

  void internalStatusCallback(const std::string &request_id, const temoto_resource_registrar::Status &status)
  {
    TEMOTO_INFO_STREAM_("internalStatusCallback"
                    << " - " << id() << " request: " << request_id);

    TEMOTO_INFO_STREAM_("Determinging if client cas callback for id " << request_id << " nr of callbacks: " << status_callbacks_.size());
    if (hasRegisteredCb(request_id))
    {
      TEMOTO_INFO_STREAM_("!Executing user CB!");
      typename ServiceClass::Request request = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Request>(status.serialised_request_);
      typename ServiceClass::Response response = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(status.serialised_response_);

      ServiceClass query;
      query.request = request;
      query.response = response;

      query.response.temoto_metadata.request_id = status.id_;

      status_callbacks_[request_id](query, status);

      //user_status_cb_(query, status); // TODO: needs exception handling
    }
  }

  bool hasRegisteredCb(const std::string &request_id) const
  {
    TEMOTO_INFO_STREAM_("hasRegisteredCb "
                    << " - " << id() << " request: " << request_id);
    return status_callbacks_.count(request_id) > 0;
  }

protected:
private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;

  //UserStatusCb user_status_cb_;
  std::unordered_map<std::string, UserStatusCb> status_callbacks_;
};

#endif