#include "rr/ros2_resource_registrar.h"

#include "tutorial_interfaces/srv/load_component.hpp"

std::string rrName = "AgentRR";

std::string latestId = "";

int main(int argc, char **argv)
{
  auto loadCb = [&](const std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Request> req, std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Response> res) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "load rr");
  };

  auto unloadCb = [&](const std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Request> req, std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Response> res) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "unload rr");
  };

  auto statusCb = [&](const std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Request> req,
                      std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Response> res,
                      const temoto_resource_registrar::Status &status) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status rr");
  };

  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "in main");
  auto rr = std::make_shared<temoto_resource_registrar::ResourceRegistrarRos2>(rrName);
  rr->init();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inited rr");

  auto server = std::make_unique<Ros2Server<tutorial_interfaces::srv::LoadComponent>>("resourceServer",
                                                                                      loadCb,
                                                                                      unloadCb,
                                                                                      statusCb);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "reg server");
  rr->registerServer(std::move(server));

  rclcpp::spin(rr);
  rclcpp::shutdown();
}