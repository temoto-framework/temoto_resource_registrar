#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

#include <boost/thread/thread.hpp>

#include "rr/ros2_resource_registrar.h"

#include "rr_interfaces/srv/load_component.hpp"
#include "rr_interfaces/srv/counter_service.hpp"

std::string rrName = "AgentRR";

std::string latestId = "";

std::shared_ptr<temoto_resource_registrar::ResourceRegistrarRos2> rr;

int main(int argc, char **argv)
{
  auto statusCallback = [&](const std::shared_ptr<rr_interfaces::srv::CounterService::Request> req,
                            std::shared_ptr<rr_interfaces::srv::CounterService::Response> res,
                            const temoto_resource_registrar::Status &status)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "statusCallback");

    for (const auto &i : rr->getServerQueries<rr_interfaces::srv::LoadComponent>("resourceServer"))
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "got a component!");

      auto r = rr->getRosChildQueries<rr_interfaces::srv::CounterService>(latestId, "counterServer");
      for (const auto &j : r) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Q id: %s", j.first.c_str());
      }

    }
  };

  auto loadCb = [&](const std::shared_ptr<rr_interfaces::srv::LoadComponent::Request> req,
                    std::shared_ptr<rr_interfaces::srv::LoadComponent::Response> res)
  {
    if (req->load_target == "counterServer")
    {
      auto request = std::make_shared<rr_interfaces::srv::CounterService::Request>();
      request->start_point = 1;
      auto res = rr->call<rr_interfaces::srv::CounterService>("ProducerRR", "counterServer", request, statusCallback);
      latestId = res.response() -> temoto_metadata.request_id;
    }
    res->load_message = req->load_target;
  };

  auto unloadCb = [&](const std::shared_ptr<rr_interfaces::srv::LoadComponent::Request> req,
                      std::shared_ptr<rr_interfaces::srv::LoadComponent::Response> res)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "unload rr");
  };

  auto statusCb = [&](const std::shared_ptr<rr_interfaces::srv::LoadComponent::Request> req,
                      std::shared_ptr<rr_interfaces::srv::LoadComponent::Response> res,
                      const temoto_resource_registrar::Status &status)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status rr");
  };

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "in main");
  rr = std::make_shared<temoto_resource_registrar::ResourceRegistrarRos2>(rrName);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inited rr");

  auto server = std::make_unique<Ros2Server<rr_interfaces::srv::LoadComponent>>("resourceServer",
                                                                                      rr,
                                                                                      loadCb,
                                                                                      unloadCb,
                                                                                      statusCb);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "reg server");

  rr->init();
  rr->registerServer(std::move(server));
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "reg server done");

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "spinning rr");

  exec.add_node(rr);
  exec.spin();
  //rclcpp::spin(rr);
  rclcpp::shutdown();
}