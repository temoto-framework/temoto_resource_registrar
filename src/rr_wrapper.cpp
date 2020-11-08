#include "ros/ros.h"
#include "rr_server_derivate.cpp"
#include "temoto_resource_registrar/TestSrv.h"
#include <temoto_resource_registrar/rr_base.h>

void myLoadCallback()
{
  ROS_INFO("Load callback");
};

void myUnLoadCallback()
{
  ROS_INFO("UnLoad callback");
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "temoto_rr");
  ros::NodeHandle nh;

  temoto_resource_registrar::RrBase rr;

  rr.addServer(std::make_unique<temoto_resource_registrar::RrServerBase>("empty_server", "string"));

  rr.addServer(std::make_unique<ServerDerivedRos<temoto_resource_registrar::TestSrv>>("pipe_server", &myLoadCallback, &myUnLoadCallback));

  ROS_INFO("ready for RR to do things");

  rr.print();

  ros::spin();

  return 0;
}