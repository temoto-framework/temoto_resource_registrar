#include <iostream>

#include "rr/ros1_resource_registrar.h"

#include "temoto_resource_registrar/CounterService.h"

std::string rrName = "ProducerRR";
temoto_resource_registrar::ResourceRegistrarRos1 rr(rrName);

void RtLoadCB(temoto_resource_registrar::CounterService::Request &req, temoto_resource_registrar::CounterService::Response &res)
{
  ROS_INFO("IN LOAD CB CounterService");
}

void RtUnloadCB(temoto_resource_registrar::CounterService::Request &req, temoto_resource_registrar::CounterService::Response &res)
{
  ROS_INFO("IN UNLOAD CB CounterService");
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting up producer...");
  ros::init(argc, argv, "producer_thing");
  rr.init();

  auto server = std::make_unique<Ros1Server<temoto_resource_registrar::CounterService>>(rrName + "_counterServer", &RtLoadCB, &RtUnloadCB);
  rr.registerServer(std::move(server));

  ROS_INFO("spinning....");
  ros::spin();

  ROS_INFO("Exiting producer...");
}