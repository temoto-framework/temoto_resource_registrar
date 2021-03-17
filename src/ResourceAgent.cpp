#include "rr/ros1_resource_registrar.h"

#include "temoto_resource_registrar/CounterService.h"
#include "temoto_resource_registrar/LoadComponent.h"

std::string rrName = "AgentRR";

temoto_resource_registrar::ResourceRegistrarRos1 rr(rrName);

void statusCallback(temoto_resource_registrar::CounterService msg, temoto_resource_registrar::StatusTodo status)
{
  ROS_INFO_STREAM("-----------------------------------IN " << __func__);
  
}

void RtM1LoadCB(temoto_resource_registrar::LoadComponent::Request &req, temoto_resource_registrar::LoadComponent::Response &res)
{
  ROS_INFO("------------------------");
  ROS_INFO("IN LOAD CB");

  temoto_resource_registrar::CounterService counterSrv;
  counterSrv.request.startPoint = 1;

  Ros1Query<temoto_resource_registrar::LoadComponent> parentQuery(res.TemotoMetadata);

  rr.call<temoto_resource_registrar::CounterService>("ProducerRR", "counterServer", counterSrv, &(parentQuery), statusCallback);

  parentQuery.rosQuery(req, res);

  res.loadMessage = req.loadTarget;
  ROS_INFO("------------------------");
}

void RtM1UnloadCB(temoto_resource_registrar::LoadComponent::Request &req, temoto_resource_registrar::LoadComponent::Response &res)
{
  ROS_INFO("IN UNLOAD CB");
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting up agent.........");
  ros::init(argc, argv, "agent_thing");

  rr.init();

  auto server = std::make_unique<Ros1Server<temoto_resource_registrar::LoadComponent>>(rrName + "_resourceServer", &RtM1LoadCB, &RtM1UnloadCB);
  rr.registerServer(std::move(server));

  ROS_INFO("spinning....");
  ros::spin();

  ROS_INFO("Exiting agent...");
}