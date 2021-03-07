#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_CLIENT_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_CLIENT_H

#include "ros/ros.h"

#include "rr/ros1_query.h"

template <class ServiceClass>
class Ros1Client : public temoto_resource_registrar::RrClientBase
{
public:
  Ros1Client(const std::string &name) : temoto_resource_registrar::RrClientBase(name)
  {
    ROS_INFO_STREAM("INIT CLIENT..." << name);

    client_ = nh_.serviceClient<ServiceClass>(name);
    ROS_INFO("INIT CLIENT DONE");
  }

  void invoke(ServiceClass &request)
  {
    ROS_INFO_STREAM("Time to invoke!!! Target: " << this->id());
    if (client_.call(request))
    {
      std::string requestId = request.response.TemotoMetadata.requestId;
      ROS_INFO_STREAM("OK " << requestId);
    }
    else
    {
      ROS_INFO("FAIL");
    }
  }

  void invoke(Ros1Query<typename ServiceClass::Request, typename ServiceClass::Response> &wrappedRequest)
  {
    ServiceClass sc;
    sc.request = wrappedRequest.request();
    sc.response = wrappedRequest.response();
    invoke(sc);

    Ros1Query<typename ServiceClass::Request, typename ServiceClass::Response> ref2(sc.request, sc.response);
    wrappedRequest = ref2;
  }

protected:
private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
};

#endif