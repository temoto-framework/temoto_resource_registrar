#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_CLIENT_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_CLIENT_H

#include "ros/ros.h"

template <class ServiceClass>
class Ros1Client : public temoto_resource_registrar::RrClientBase
{
public:
  Ros1Client(const std::string &name);

  void invoke(ServiceClass &request);

protected:
private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
};

#endif