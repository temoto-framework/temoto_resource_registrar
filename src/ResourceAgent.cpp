#include "rr/Ros1ResourceRegistrar.cpp"

#include "temoto_resource_registrar/LoadComponent.h"

std::string rrName = "AgentRR";

Ros1ResourceRegistrar<temoto_resource_registrar::RrServerBase, temoto_resource_registrar::RrClientBase> rr(rrName);

void RtM1LoadCB(temoto_resource_registrar::LoadComponent &query){
  ROS_INFO("IN LOAD CB");
}

void RtM1UnloadCB(temoto_resource_registrar::LoadComponent &query){
  ROS_INFO("IN UNLOAD CB");
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting up agent...");
  ros::init(argc, argv, "agent_thing");

  auto server = std::make_unique<Ros1Server<temoto_resource_registrar::LoadComponent,
                                            temoto_resource_registrar::LoadComponent::Request,
                                            temoto_resource_registrar::LoadComponent::Response>>(rrName  + "_resourceServer", &RtM1LoadCB, &RtM1UnloadCB);
  rr.registerServer(std::move(server));

  ROS_INFO("spinning...");
  ros::spin();

  ROS_INFO("Exiting agent...");
}