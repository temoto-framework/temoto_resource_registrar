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

  Ros1Client(const std::string &name) : temoto_resource_registrar::RrClientBase(name)
  {
    client_ = nh_.serviceClient<ServiceClass>(name);
  }

  /**
 * @brief Invokes a call to the Ros1Server responsible for serving requests of that type. 
 * The request elements are modified to provide a response
 * 
 * @param request - A user defined ROS1 srv type request.
 */
  void invoke(ServiceClass &request)
  {
    if (client_.call(request))
    {
      ROS_INFO_STREAM("invoke OK " << request.response.TemotoMetadata.requestId);
    }
    else
    {
      ROS_INFO("FAIL");
    }
  }

  /**
 * @brief Invokes a call to the Ros1Server responsible for serving requests of that type. 
 * The request elements are modified to provide a response
 * 
 * @param wrappedRequest - A user defined Ros1Query type request
 */
  void invoke(Ros1Query<ServiceClass> &wrappedRequest)
  {
    ServiceClass servCall = wrappedRequest.rosQuery();

    invoke(servCall);

    wrappedRequest = Ros1Query<ServiceClass>(servCall);
  }

  void registerUserStatusCb(const UserStatusCb &user_status_cb)
  {
    ROS_INFO_STREAM("registerUserStatusCb "
                    << " - " << id());
    user_status_cb_ = user_status_cb;
  }

  void internalStatusCallback(const temoto_resource_registrar::Status &status)
  {
    ROS_INFO_STREAM("internalStatusCallback"

                    << " - " << id());
    if (hasRegisteredCb())
    {

      auto idstr = rr_catalog_->getOriginQueryId(status.id_);

      ROS_INFO_STREAM("CB exists " << status.id_ << " --- " << idstr);

      auto container = rr_catalog_->findOriginalContainer(idstr);
      if (!container.empty_) {
        typename ServiceClass::Request request = MessageSerializer::deSerializeMessage<typename ServiceClass::Request>(container.rawRequest_);
        typename ServiceClass::Response response = MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(container.rawQuery_);

        ServiceClass query; // TODO: get the query from the rr_catalog;
        query.request = request;
        query.response = response;
        user_status_cb_(query, status); // TODO: needs exception handling
      } else {
        ROS_INFO_STREAM("NO CONTAINER");
      }
        
    }
  }

  bool hasRegisteredCb()
  {
    ROS_INFO_STREAM("hasRegisteredCb "
                    << " - " << id());
    return user_status_cb_ != NULL;
  }

protected:
private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;

  UserStatusCb user_status_cb_;
};

#endif