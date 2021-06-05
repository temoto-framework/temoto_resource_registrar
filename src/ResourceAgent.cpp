#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

#include <boost/thread/thread.hpp>

#include "rr/ros2_resource_registrar.h"

#include "tutorial_interfaces/srv/load_component.hpp"
#include "tutorial_interfaces/srv/counter_service.hpp"

std::string rrName = "AgentRR";

std::string latestId = "";
std::string id = "";

std::shared_ptr<temoto_resource_registrar::ResourceRegistrarRos2> rr;

int main(int argc, char **argv)
{
  auto statusCallback = [&](const std::shared_ptr<tutorial_interfaces::srv::CounterService::Request> req,
                            std::shared_ptr<tutorial_interfaces::srv::CounterService::Response> res,
                            const temoto_resource_registrar::Status &status)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status rr");
  };

  auto loadCb = [&](const std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Request> req,
                    std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Response> res)
  {
    if (req->load_target == "counterServer")
    {
      auto request = std::make_shared<tutorial_interfaces::srv::CounterService::Request>();
      request->start_point = 1;
      rr->call<tutorial_interfaces::srv::CounterService>("ProducerRR", "counterServer", request, statusCallback);
    }
    res->load_message = req->load_target;
  };

  auto unloadCb = [&](const std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Request> req,
                      std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Response> res)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "unload rr");
  };

  auto statusCb = [&](const std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Request> req,
                      std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Response> res,
                      const temoto_resource_registrar::Status &status)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status rr");
  };

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  //exec.add_node(rr);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "in main");
  rr = std::make_shared<temoto_resource_registrar::ResourceRegistrarRos2>(rrName);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inited rr");

  auto server = std::make_unique<Ros2Server<tutorial_interfaces::srv::LoadComponent>>("resourceServer",
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