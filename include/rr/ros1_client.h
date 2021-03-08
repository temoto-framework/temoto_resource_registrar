#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_CLIENT_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_CLIENT_H

#include "ros/ros.h"

#include "rr/ros1_query.h"

/**
 * @brief A wrapper class for the temoto_resource_registrar::RRClientBase. Provides templating to support multiple message types.
 * 
 * @tparam ServiceClass - type of message this client supports
 */
template <class ServiceClass>
class Ros1Client : public temoto_resource_registrar::RrClientBase
{
public:
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

protected:
private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
};

#endif