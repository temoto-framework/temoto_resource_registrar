#include "rr/ros2_resource_registrar.h"
#include "rr/ros2_client.h"
#include "rr/ros2_server.h"
#include "rr/ros2_query.h"

#include "rclcpp/rclcpp.hpp"

#include "tutorial_interfaces/srv/load_component.hpp"

std::string rrName = "ConsumerRR";


int main(int argc, char **argv)
{
  
  auto loadCb = [&](const std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Request> req, std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Response> res) {

  };

  auto unloadCb = [&](const std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Request> req, std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Response> res) {

  };

  auto statusCb = [&](const std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Request> req,
                      std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Response> res,
                      const temoto_resource_registrar::Status &status) {

  };

  
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "in main"); 
  auto rr = std::make_shared<temoto_resource_registrar::ResourceRegistrarRos2>(rrName);
  rr->init();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inited rr"); 

  //Ros2Client<tutorial_interfaces::srv::LoadComponent> r2("test", "val");
  //Ros2Query<tutorial_interfaces::srv::LoadComponent> q();

  //Ros2Server<tutorial_interfaces::srv::LoadComponent> s("sss", loadCb, unloadCb, statusCb);

  auto request = std::make_shared<tutorial_interfaces::srv::LoadComponent::Request>();
  request -> load_target = "CounterService";

  rr->call<std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Request>>("AgentRR", "resourceServer", request);

  rclcpp::shutdown();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "shutdown"); 
  return 0;
}