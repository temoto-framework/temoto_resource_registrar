#include "rr/ros1_resource_registrar.h"

#include "temoto_resource_registrar/CounterService.h"
#include "temoto_resource_registrar/LoadComponent.h"

std::string rrName = "AgentRR";

temoto_resource_registrar::ResourceRegistrarRos1 rr(rrName);

void statusCallback(temoto_resource_registrar::CounterService msg, temoto_resource_registrar::Status status)
{
  ROS_INFO_STREAM("-----------------------------------IN " << __func__ << " - " << status.serialisedRequest_.size() << " - " << msg.request.temotoMetadata.originRr);

  std::string n = rrName + "_resourceServer";
  for (temoto_resource_registrar::LoadComponent const &i : rr.getServerQueries<temoto_resource_registrar::LoadComponent>(n))
  {
    ROS_INFO_STREAM(i.response.temotoMetadata.requestId);
    ROS_INFO_STREAM(i.response.loadMessage);
  }
}

void RtM1LoadCB(temoto_resource_registrar::LoadComponent::Request &req, temoto_resource_registrar::LoadComponent::Response &res)
{
  ROS_INFO("IN LOAD CB ------------------------");

  ROS_INFO_STREAM("1" << req.loadTarget);
  if (req.loadTarget == "CounterService")
  {
    ROS_INFO_STREAM("2" << req.loadTarget);
    temoto_resource_registrar::CounterService counterSrv;
    counterSrv.request.startPoint = 1;

    Ros1Query<temoto_resource_registrar::LoadComponent> parentQuery(req.temotoMetadata, res.temotoMetadata);

    rr.call<temoto_resource_registrar::CounterService>("ProducerRR", "counterServer", counterSrv, &(parentQuery), statusCallback);
    ROS_INFO_STREAM("3" << req.loadTarget);
    parentQuery.rosQuery(req, res);
    ROS_INFO_STREAM("4" << req.loadTarget);
  }
  ROS_INFO_STREAM("5" << req.loadTarget);
  res.loadMessage = req.loadTarget;
  ROS_INFO("IN LOAD CB------------------------");
}

void RtM1UnloadCB(temoto_resource_registrar::LoadComponent::Request &req, temoto_resource_registrar::LoadComponent::Response &res)
{
  ROS_INFO("IN UNLOAD CB");
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting up agent.........");
  ros::init(argc, argv, "agent_thing");

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  rr.init();

  auto server = std::make_unique<Ros1Server<temoto_resource_registrar::LoadComponent>>(rrName + "_resourceServer", &RtM1LoadCB, &RtM1UnloadCB);
  rr.registerServer(std::move(server));

  ROS_INFO("spinning....");
  //ros::spin();
  ros::waitForShutdown();

  ROS_INFO("Exiting agent...");
}