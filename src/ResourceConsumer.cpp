#include "rr/ros2_resource_registrar.h"
#include "rr/ros2_client.h"
#include "rr/ros2_server.h"
#include "rr/ros2_query.h"

#include "rclcpp/rclcpp.hpp"

#include "rr_interfaces/srv/load_component.hpp"

std::string rrName = "ConsumerRR";

int counter = 0;
int shutdownCounter = 5;
std::string loadId = "";

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "in main");
  auto rr = std::make_shared<temoto_resource_registrar::ResourceRegistrarRos2>(rrName);
  rr->init();

  auto statusCb = [&](const std::shared_ptr<rr_interfaces::srv::LoadComponent::Request> &req,
                      std::shared_ptr<rr_interfaces::srv::LoadComponent::Response> res,
                      const temoto_resource_registrar::Status &status) {
    counter++;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Client cb... %i", counter);

    if (counter == shutdownCounter)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "counter reached, unloading: " + loadId);
      bool res = rr->unload("AgentRR", loadId);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "unload result: %d", res);
    }
  };

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inited rr");


  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "constructing request");
  auto request = std::make_shared<rr_interfaces::srv::LoadComponent::Request>();
  request->load_target = "counterServer";

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executing call");

  auto res = rr->call<rr_interfaces::srv::LoadComponent>("AgentRR", "resourceServer", request, statusCb);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "call done");

  loadId = res.response() -> temoto_metadata.request_id;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), loadId);

  exec.add_node(rr);
  exec.spin();

  rclcpp::shutdown();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "shutdown");
  return 0;
}