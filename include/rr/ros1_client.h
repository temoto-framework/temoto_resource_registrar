#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_CLIENT_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_CLIENT_H

#include "ros/ros.h"
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
      ROS_INFO_STREAM("OK");
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

    
    ServiceClass sc;
    sc.request = wrappedRequest.request();
    sc.response = wrappedRequest.response();
    invoke(sc);

    Ros1Query<ServiceClass> ref2(sc.request, sc.response);
    wrappedRequest = ref2;
  }

  void registerUserStatusCb(const UserStatusCb& user_status_cb)
  {
    user_status_cb_ = user_status_cb;
  }

  void internalStatusCallback(const temoto_resource_registrar::Status& status)
  {
    ServiceClass query; // TODO: get the query from the rr_catalog;
    user_status_cb_(query, status); // TODO: needs exception handling
  }

protected:
private:

  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  UserStatusCb user_status_cb_;
};

#endif