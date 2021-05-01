#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

#include "rr/ros1_resource_registrar.h"

#include "temoto_resource_registrar/CounterService.h"
#include "temoto_resource_registrar/temoto_error.h"

#include <boost/thread/thread.hpp>

std::string rrName = "ProducerRR";
temoto_resource_registrar::ResourceRegistrarRos1 rr(rrName);

bool loaded = false;
std::string id = "";

void caller(int loopNr)
{
  for (int n = 0; n < loopNr; ++n)
  {
    std::string message = "some message i need to see";
    ROS_INFO("caller...");
    rr.sendStatus(id, {temoto_resource_registrar::Status::State::FATAL, id, message});
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

/*
void RtLoadCB(temoto_resource_registrar::CounterService::Request &req, temoto_resource_registrar::CounterService::Response &res)
{
  ROS_INFO_STREAM("IN LOAD CB CounterService " << res.temotoMetadata.requestId);
  id = res.temotoMetadata.requestId;

  //throw resource_registrar::TemotoErrorStack("producer error", "ResourceProducer");

  //auto fa = std::async(std::launch::async, caller);

  boost::thread thread_b(caller, 5);
}

void RtUnloadCB(temoto_resource_registrar::CounterService::Request &req, temoto_resource_registrar::CounterService::Response &res)
{
  ROS_INFO_STREAM("IN UNLOAD CB CounterService. ID: " << req << " - " << res);
}

void RtStatusCb(temoto_resource_registrar::CounterService::Request &req, temoto_resource_registrar::CounterService::Response &res, const temoto_resource_registrar::Status &status)
{
  ROS_WARN_STREAM("STATUS!");
}
*/

int main(int argc, char **argv)
{

  auto loadCb = [&](temoto_resource_registrar::CounterService::Request &req, temoto_resource_registrar::CounterService::Response &res) {
    ROS_INFO_STREAM("IN LOAD CB CounterService " << res.temotoMetadata.requestId);
    id = res.temotoMetadata.requestId;

    boost::thread thread_b(caller, 5);
  };

  auto unloadCb = [&](temoto_resource_registrar::CounterService::Request &req, temoto_resource_registrar::CounterService::Response &res) {
    ROS_INFO_STREAM("IN UNLOAD CB CounterService. ID: " << req << " - " << res);
  };

  auto statusCb = [&](temoto_resource_registrar::CounterService::Request &req, temoto_resource_registrar::CounterService::Response &res, const temoto_resource_registrar::Status &status) {
    ROS_ERROR_STREAM("Status CB called");
  };

  ROS_INFO("Starting up producer...");
  ros::init(argc, argv, "producer_thing");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  rr.init();

  auto server = std::make_unique<Ros1Server<temoto_resource_registrar::CounterService>>("counterServer", loadCb, unloadCb, statusCb);
  rr.registerServer(std::move(server));

  ROS_INFO("spinning....");

  /*while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (loaded) {
      auto fa = std::async(std::launch::async, caller);
      loaded = false;
    }
  }*/
  //ros::spin();
  ros::waitForShutdown();

  ROS_INFO("Exiting producer...");
}