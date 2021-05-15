#include "rr/ros1_resource_registrar.h"

#include "temoto_resource_registrar/CounterService.h"
#include "temoto_resource_registrar/LoadComponent.h"

std::string rrName = "AgentRR";

std::string latestId = "";

temoto_resource_registrar::ResourceRegistrarRos1 rr(rrName);

void statusCallback(temoto_resource_registrar::CounterService msg, temoto_resource_registrar::Status status)
{
  ROS_INFO_STREAM("-----------------------------------IN " << __func__ << " - " << status.serialised_request_.size() << " - " << msg.request.temoto_metadata.origin_rr);

  for (temoto_resource_registrar::LoadComponent const &i : rr.getServerQueries<temoto_resource_registrar::LoadComponent>("resourceServer"))
  {
    ROS_INFO_STREAM("<<<<<<< " << i.response.temoto_metadata.request_id);
    ROS_INFO_STREAM("<<<<<<< " << i.response.load_message);

    std::map<std::string, temoto_resource_registrar::CounterService> r = rr.getRosChildQueries<temoto_resource_registrar::CounterService>(latestId, "counterServer");
    ROS_INFO_STREAM("<<<<<<<<<<<<<<<<<<dependency result>>>>>>>>>>>>>>>>>>>>>" << r.size());
    for (const auto &i : r) {
      ROS_ERROR_STREAM("Query IDS:" << i.first);
    }
  }
}

int main(int argc, char **argv)
{

  auto loadCb = [&](temoto_resource_registrar::LoadComponent::Request &req, temoto_resource_registrar::LoadComponent::Response &res) {
    ROS_INFO("IN LOAD CB ------------------------");

    ROS_INFO_STREAM("1" << req.load_target);
    if (req.load_target == "CounterService")
    {
      ROS_INFO_STREAM("2" << req.load_target);
      temoto_resource_registrar::CounterService counterSrv;
      counterSrv.request.start_point = 1;

      rr.call<temoto_resource_registrar::CounterService>("ProducerRR", "counterServer", counterSrv, statusCallback);
      ROS_INFO_STREAM("3" << req.load_target);
      latestId = res.temoto_metadata.request_id;
    }
    ROS_INFO_STREAM("5" << req.load_target);
    res.load_message = req.load_target;
    ROS_INFO("IN LOAD CB------------------------");
  };

  auto unloadCb = [&](temoto_resource_registrar::LoadComponent::Request &req, temoto_resource_registrar::LoadComponent::Response &res) {
    ROS_INFO("IN UNLOAD CB");
  };

  auto statusCb = [&](temoto_resource_registrar::LoadComponent::Request &req, temoto_resource_registrar::LoadComponent::Response &res, const temoto_resource_registrar::Status &status) {
    ROS_ERROR_STREAM("Status CB called");
  };

  ROS_INFO("Starting up agent.........");
  ros::init(argc, argv, "agent_thing");

  ros::AsyncSpinner spinner(10); // Use 10 threads
  spinner.start();

  rr.init();

  auto server = std::make_unique<Ros1Server<temoto_resource_registrar::LoadComponent>>("resourceServer", loadCb, unloadCb, statusCb);
  rr.registerServer(std::move(server));

  ROS_INFO("spinning....");
  //ros::spin();
  ros::waitForShutdown();

  ROS_INFO("Exiting agent...");
}