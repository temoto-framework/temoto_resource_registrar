#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

#include <boost/thread/thread.hpp>

#include "rr/ros2_resource_registrar.h"

#include "tutorial_interfaces/srv/counter_service.hpp"

std::string rrName = "ProducerRR";

std::string latestId = "";
std::string id = "";

std::shared_ptr<temoto_resource_registrar::ResourceRegistrarRos2> rr;

void caller(int loopNr)
{
  for (int n = 0; n < loopNr; ++n)
  {
    std::string message = "some message i need to see";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "caller...");
    rr->sendStatus(id, {temoto_resource_registrar::Status::State::FATAL, id, message});
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

int main(int argc, char **argv)
{
  auto loadCb = [&](const std::shared_ptr<tutorial_interfaces::srv::CounterService::Request> req, std::shared_ptr<tutorial_interfaces::srv::CounterService::Response> res) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "load rr");

    id = res->temoto_metadata.request_id;

    boost::thread thread_b(caller, 6);
  };

  auto unloadCb = [&](const std::shared_ptr<tutorial_interfaces::srv::CounterService::Request> req, std::shared_ptr<tutorial_interfaces::srv::CounterService::Response> res) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "unload rr");
  };

  auto statusCb = [&](const std::shared_ptr<tutorial_interfaces::srv::CounterService::Request> req,
                      std::shared_ptr<tutorial_interfaces::srv::CounterService::Response> res,
                      const temoto_resource_registrar::Status &status) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status rr");

    //boost::thread thread_b(caller, 5);
  };

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  //exec.add_node(rr);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "in main");
  rr = std::make_shared<temoto_resource_registrar::ResourceRegistrarRos2>(rrName);

  

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inited rr");

  auto server = std::make_unique<Ros2Server<tutorial_interfaces::srv::CounterService>>("counterServer",
                                                                                      rr,
                                                                                      loadCb,
                                                                                      unloadCb,
                                                                                      statusCb);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "reg server");

  rr->init();
  rr->registerServer(std::move(server));
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "reg server done");

  //rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "spinning rr");

  exec.add_node(rr);
  exec.spin();
  //rclcpp::spin(rr);
  rclcpp::shutdown();
}