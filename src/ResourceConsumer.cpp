#include "rr/Ros1ResourceRegistrar.cpp"

#include "temoto_resource_registrar/LoadComponent.h"

std::string rrName = "ConsumerRR";

Ros1ResourceRegistrar<temoto_resource_registrar::RrServerBase, temoto_resource_registrar::RrClientBase> rr(rrName);

int main(int argc, char **argv)
{
  ROS_INFO("Starting up consumer...");
  ros::init(argc, argv, "consumer_thing");

  temoto_resource_registrar::LoadComponent loadCall;
  loadCall.request.loadTarget = "CounterService";

  rr.call<Ros1Client<temoto_resource_registrar::LoadComponent>, temoto_resource_registrar::LoadComponent>("AgentRR", "resourceServer", loadCall);

  ROS_INFO("Exiting consumer...");
}