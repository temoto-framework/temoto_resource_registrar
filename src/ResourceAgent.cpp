#include "rr/ros1_resource_registrar.h"

#include "temoto_resource_registrar/LoadComponent.h"

std::string rrName = "AgentRR";
void RtM1LoadCB(temoto_resource_registrar::LoadComponent::Request &req, temoto_resource_registrar::LoadComponent::Response &res)
{
  ROS_INFO("IN LOAD CB");

  res.loadMessage = req.loadTarget;
}

void RtM1UnloadCB(temoto_resource_registrar::LoadComponent::Request &req, temoto_resource_registrar::LoadComponent::Response &res)
{
  ROS_INFO("IN UNLOAD CB");
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting up agent...");
  ros::init(argc, argv, "agent_thing");

  temoto_resource_registrar::ResourceRegistrarRos1 rr(rrName);

  auto server = std::make_unique<Ros1Server<temoto_resource_registrar::LoadComponent,
                                            temoto_resource_registrar::LoadComponent::Request,
                                            temoto_resource_registrar::LoadComponent::Response>>(rrName + "_resourceServer", &RtM1LoadCB, &RtM1UnloadCB);
  rr.registerServer(std::move(server));

  ROS_INFO("spinning....");
  ros::spin();

  ROS_INFO("Exiting agent...");
}