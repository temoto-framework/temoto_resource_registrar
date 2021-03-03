#include "rr/ros1_client.h"

template <class ServiceClass>
Ros1Client<ServiceClass>::Ros1Client(const std::string &name) : temoto_resource_registrar::RrClientBase(name)
{
  ROS_INFO_STREAM("INIT CLIENT..." << name);

  client_ = nh_.serviceClient<ServiceClass>(name);
  ROS_INFO("INIT CLIENT DONE");
}

template <class ServiceClass>
void Ros1Client<ServiceClass>::invoke(ServiceClass &request)
{
  ROS_INFO_STREAM("Time to invoke!!! Target: " << this->id());
  if (client_.call(request))
  {
    ROS_INFO("OK");
  }
  else
  {
    ROS_INFO("FAIL");
  }
}
