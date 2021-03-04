#include "rr/ros1_resource_registrar.h"

#include "temoto_resource_registrar/LoadComponent.h"

std::string rrName = "ConsumerRR";



int main(int argc, char **argv)
{
  ROS_INFO("Starting up consumer...");
  ros::init(argc, argv, "consumer_thing");

  temoto_resource_registrar::ResourceRegistrarRos1 rr(rrName);

  temoto_resource_registrar::LoadComponent loadCall;
  loadCall.request.loadTarget = "CounterService";

  rr.call<temoto_resource_registrar::LoadComponent>("AgentRR", "resourceServer", loadCall);

  std::string load1Id = loadCall.response.TemotoMetadata.requestId;

  ROS_INFO_STREAM("OUTPUT RESULT: " << loadCall.response.loadMessage << "; id: " << load1Id);

  rr.unload("AgentRR", load1Id);

  temoto_resource_registrar::LoadComponent loadCall2;
  loadCall2.request.loadTarget = "TimeService";

  rr.call<temoto_resource_registrar::LoadComponent>("AgentRR", "resourceServer", loadCall2);

  std::string load2Id = loadCall2.response.TemotoMetadata.requestId;

  ROS_INFO_STREAM("OUTPUT RESULT: " << loadCall2.response.loadMessage << "; id: " << load2Id);

  //rr.unload("AgentRR", load2Id);

  ROS_INFO("Exiting consumer....");
}