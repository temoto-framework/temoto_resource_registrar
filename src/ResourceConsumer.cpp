#include "rr/ros1_resource_registrar.h"

#include "temoto_resource_registrar/LoadComponent.h"

std::string rrName = "ConsumerRR";
temoto_resource_registrar::ResourceRegistrarRos1 rr(rrName);

void statusCallback(temoto_resource_registrar::LoadComponent msg, temoto_resource_registrar::Status status)
{
  ROS_INFO_STREAM("-----------------------------------IN " << __func__ << " - " << status.serialisedRequest_.size() << " - " << msg.request.TemotoMetadata.originRr);
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting up consumer...");
  ros::init(argc, argv, "consumer_thing");

  rr.init();

  /*std::vector<std::string> dependencies;
  dependencies.push_back("rr1;;first");
  dependencies.push_back("rr1;;second");
  dependencies.push_back("rr1;;third");

  temoto_resource_registrar::LoadComponent loadCall;
  loadCall.request.loadTarget = "CounterService";
  loadCall.request.TemotoMetadata.requestId = "requestId";
  loadCall.request.TemotoMetadata.servingRr = "servingRr";
  loadCall.request.TemotoMetadata.originRr = "originRr";
  loadCall.request.TemotoMetadata.dependencies = dependencies;
  loadCall.response.loadMessage = "OK";

  ROS_INFO_STREAM("loadCall.request.loadTarget: " << loadCall.request.loadTarget);
  ROS_INFO_STREAM("loadCall.request.TemotoMetadata.requestId: " << loadCall.request.TemotoMetadata.requestId);
  ROS_INFO_STREAM("loadCall.request.TemotoMetadata.servingRr: " << loadCall.request.TemotoMetadata.servingRr);
  ROS_INFO_STREAM("loadCall.request.TemotoMetadata.originRr: " << loadCall.request.TemotoMetadata.originRr);
  ROS_INFO_STREAM("loadCall.response.loadMessage: " << loadCall.response.loadMessage);

  ROS_INFO_STREAM("dependency size: " << loadCall.request.TemotoMetadata.dependencies.size());
  for (const auto &el : loadCall.request.TemotoMetadata.dependencies)
  {
    ROS_INFO_STREAM("dependency: " << el);
  }

  ROS_INFO_STREAM("------------------------------------------------------------");

  Ros1Query<temoto_resource_registrar::LoadComponent> rosQuery(loadCall);

  rosQuery.request().TemotoMetadata.requestId = "modified";

  rosQuery.includeDependency("a", "b");
  rosQuery.setId("actualId");
  rosQuery.setRr("actualServingRr");
  rosQuery.setOrigin("actualOriginRr");

  temoto_resource_registrar::LoadComponent newLoadCall = rosQuery.rosQuery();

  for (const auto &el : rosQuery.dependencies())
  {
    ROS_INFO_STREAM("dependency: " << el.first << " - " << el.second);
  }

  ROS_INFO_STREAM("------------------------------------------------------------");

  ROS_INFO_STREAM("loadCall.request.loadTarget: " << newLoadCall.request.loadTarget);
  ROS_INFO_STREAM("loadCall.request.TemotoMetadata.requestId: " << newLoadCall.request.TemotoMetadata.requestId);
  ROS_INFO_STREAM("loadCall.request.TemotoMetadata.servingRr: " << newLoadCall.request.TemotoMetadata.servingRr);
  ROS_INFO_STREAM("loadCall.request.TemotoMetadata.originRr: " << newLoadCall.request.TemotoMetadata.originRr);
  ROS_INFO_STREAM("loadCall.response.loadMessage: " << newLoadCall.response.loadMessage);

  ROS_INFO_STREAM("dependency size: " << newLoadCall.request.TemotoMetadata.dependencies.size());
  for (const auto &el : newLoadCall.request.TemotoMetadata.dependencies)
  {
    ROS_INFO_STREAM("dependency: " << el);
  }*/

  temoto_resource_registrar::LoadComponent loadCall;
  loadCall.request.loadTarget = "CounterService";

  rr.call<temoto_resource_registrar::LoadComponent>("AgentRR", "resourceServer", loadCall, NULL, statusCallback);

  std::string load1Id = loadCall.response.TemotoMetadata.requestId;

  ROS_INFO_STREAM("OUTPUT RESULT: " << loadCall.response.loadMessage << "; id: " << load1Id);

  //bool unloadRes = rr.unload("AgentRR", load1Id);
  //ROS_INFO_STREAM("Unload result: " << unloadRes);

  temoto_resource_registrar::LoadComponent loadCall2;
  loadCall2.request.loadTarget = "TimeService";

  rr.call<temoto_resource_registrar::LoadComponent>("AgentRR", "resourceServer", loadCall2);

  std::string load2Id = loadCall2.response.TemotoMetadata.requestId;

  ROS_INFO_STREAM("OUTPUT RESULT: " << loadCall2.response.loadMessage << "; id: " << load2Id);

  //rr.unload("AgentRR", load2Id);

  rr.printCatalog();

  ros::spin();

  ROS_INFO("Exiting consumer....");
}