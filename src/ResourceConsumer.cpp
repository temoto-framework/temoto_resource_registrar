#include "rr/ros2_resource_registrar.h"
#include "rr/ros2_client.h"
#include "rr/ros2_server.h"
#include "rr/ros2_query.h"

#include "rclcpp/rclcpp.hpp"

#include "tutorial_interfaces/srv/load_component.hpp"

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

  auto statusCb = [&](const std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Request> &req,
                      std::shared_ptr<tutorial_interfaces::srv::LoadComponent::Response> res,
                      const temoto_resource_registrar::Status &status) {
    counter++;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Client cb... %i", counter);

    if (counter == shutdownCounter)
    {

      //rr.printCatalog();
      //auto r = rr.getRosChildQueries<temoto_resource_registrar::LoadComponent>(loadId, "resourceServer");

      //for (const auto &el : r)
      //{
      //  ROS_INFO_STREAM("id: " << el.first << " - msg: " << el.second.request);
      //}

      //ROS_INFO_STREAM("getServerRrQueries size: " << r.size());

      //ROS_INFO_STREAM("Counter reached, unloading: " << loadId << " from: " << "AgentRR");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "counter reached, unloading: " + loadId);
      bool res = rr->unload("AgentRR", loadId);
      //ROS_INFO_STREAM("Unload result: " << res);
    }
  };

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inited rr");

  //rr->sendStatus2();

  //Ros2Client<tutorial_interfaces::srv::LoadComponent> r2("test", "val");
  //Ros2Query<tutorial_interfaces::srv::LoadComponent> q();

  //Ros2Server<tutorial_interfaces::srv::LoadComponent> s("sss", loadCb, unloadCb, statusCb);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "constructing request");
  auto request = std::make_shared<tutorial_interfaces::srv::LoadComponent::Request>();
  request->load_target = "CounterService";

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executing call");

  auto res = rr->call<tutorial_interfaces::srv::LoadComponent>("AgentRR", "resourceServer", request, statusCb);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "call done");

  std::string requestId = res.response() -> temoto_metadata.request_id;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), requestId);

  //exec.add_node(rr);
  //exec.spin();

  rclcpp::spin(rr);
  rclcpp::shutdown();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "shutdown");
  return 0;
}