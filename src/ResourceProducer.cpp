#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

#include "rr/ros1_resource_registrar.h"

#include "temoto_resource_registrar/CounterService.h"

std::string rrName = "ProducerRR";
temoto_resource_registrar::ResourceRegistrarRos1 rr(rrName);

bool loaded = false;
std::string id = "";

void caller()
{
  for (int n = 0; n < 3; ++n)
  {
    std::string message = "some message i need to see";
    ROS_INFO("caller...");
    //rr.sendStatus({temoto_resource_registrar::StatusTodo::State::FATAL, id, message});
    rr.sendStatus(id, temoto_resource_registrar::Status::FATAL, message);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void RtLoadCB(temoto_resource_registrar::CounterService::Request &req, temoto_resource_registrar::CounterService::Response &res)
{
  ROS_INFO_STREAM("IN LOAD CB CounterService " << res.TemotoMetadata.requestId);
  id = res.TemotoMetadata.requestId;
  loaded = true;
}

void RtUnloadCB(temoto_resource_registrar::CounterService::Request &req, temoto_resource_registrar::CounterService::Response &res)
{
  ROS_INFO("IN UNLOAD CB CounterService");
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting up producer...");
  ros::init(argc, argv, "producer_thing");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  rr.init();

  auto server = std::make_unique<Ros1Server<temoto_resource_registrar::CounterService>>(rrName + "_counterServer", &RtLoadCB, &RtUnloadCB);
  rr.registerServer(std::move(server));

  ROS_INFO("spinning....");


  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (loaded) {
      auto fa = std::async(std::launch::async, caller);
      loaded = false;
    }
      
  }

  ROS_INFO("Exiting producer...");
}