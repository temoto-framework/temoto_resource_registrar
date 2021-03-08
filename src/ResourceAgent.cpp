#include "rr/ros1_resource_registrar.h"

#include "temoto_resource_registrar/CounterService.h"
#include "temoto_resource_registrar/LoadComponent.h"

std::string rrName = "AgentRR";

temoto_resource_registrar::ResourceRegistrarRos1 rr(rrName);

void RtM1LoadCB(temoto_resource_registrar::LoadComponent::Request &req, temoto_resource_registrar::LoadComponent::Response &res)
{
  ROS_INFO("------------------------");
  ROS_INFO("IN LOAD CB");

  temoto_resource_registrar::CounterService counterSrv;
  counterSrv.request.startPoint = 1;

  ROS_INFO_STREAM("loadCall.request.TemotoMetadata.requestId: " << res.TemotoMetadata.requestId);
  ROS_INFO_STREAM("loadCall.request.TemotoMetadata.servingRr: " << res.TemotoMetadata.servingRr);
  ROS_INFO_STREAM("loadCall.request.TemotoMetadata.originRr: " << res.TemotoMetadata.originRr);
  ROS_INFO_STREAM("loadCall.request.TemotoMetadata.dependencies: " << res.TemotoMetadata.dependencies.size());

  temoto_resource_registrar::LoadComponent srvCall;
  srvCall.request = req;
  srvCall.response = res;
  Ros1Query<temoto_resource_registrar::LoadComponent> parentQuery(srvCall);

  ROS_INFO_STREAM("wrappedQuery.requestId: " << parentQuery.id());
  ROS_INFO_STREAM("wrappedQuery.servingRr: " << parentQuery.rr());
  ROS_INFO_STREAM("wrappedQuery.originRr: " << parentQuery.origin());

  rr.call<temoto_resource_registrar::CounterService>("ProducerRR", "counterServer", counterSrv, &(parentQuery));

  ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!parentQuery.dependencies().size(): " << parentQuery.dependencies().size());
  ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!parentQuery.dependencies().size(): " << counterSrv.response.TemotoMetadata.dependencies.size());

  temoto_resource_registrar::LoadComponent q = parentQuery.rosQuery();
  res = q.response;
  req = q.request;
  
  res.loadMessage = req.loadTarget;
  ROS_INFO("------------------------");
}

void RtM1UnloadCB(temoto_resource_registrar::LoadComponent::Request &req, temoto_resource_registrar::LoadComponent::Response &res)
{
  ROS_INFO("IN UNLOAD CB");
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting up agent...");
  ros::init(argc, argv, "agent_thing");

  rr.init();

  auto server = std::make_unique<Ros1Server<temoto_resource_registrar::LoadComponent>>(rrName + "_resourceServer", &RtM1LoadCB, &RtM1UnloadCB);
  rr.registerServer(std::move(server));

  ROS_INFO("spinning....");
  ros::spin();

  ROS_INFO("Exiting agent...");
}