#include "rr/ros1_resource_registrar.h"

#include "temoto_resource_registrar/LoadComponent.h"

std::string rrName = "ConsumerRR";
temoto_resource_registrar::ResourceRegistrarRos1 rr(rrName);

int counter = 0;
int shutdownCounter = 5;
std::string loadId = "";

void statusCallback(temoto_resource_registrar::LoadComponent msg, temoto_resource_registrar::Status status)
{
  ROS_INFO_STREAM("-----------------------------------IN " << __func__ << " - " << status.serialisedRequest_.size() << " - " << msg.request.temotoMetadata.originRr);

  counter++;

  if (counter == shutdownCounter)
  {

    auto r = rr.getRosServerRrQueries<temoto_resource_registrar::LoadComponent::Request>("AgentRR_resourceServer", rrName);

    for(const auto &el : r) {
      ROS_INFO_STREAM("id: " << el.first << " - msg: " << el.second);
    }

    ROS_INFO_STREAM("getServerRrQueries size: " << r.size());

    ROS_INFO_STREAM("Counter reached, unloading: " << loadId << " from: "
                                                   << "AgentRR");
    bool res = rr.unload("AgentRR", loadId);
    ROS_INFO_STREAM("Unload result: " << res);
  }
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting up consumer...");
  ros::init(argc, argv, "consumer_thing");
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
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

  ROS_INFO_STREAM("Calling server with error expectation...");
  try {
    rr.call<temoto_resource_registrar::LoadComponent>("AgentRR", "resourceServer", loadCall, statusCallback);

    loadId = loadCall.response.temotoMetadata.requestId;
  }
  catch (const resource_registrar::TemotoErrorStack &e)
  {
    ROS_WARN_STREAM("Something bad happened. What is the cause i wonder? : " << e.getMessage());
  }

  /*ROS_INFO_STREAM("OUTPUT RESULT: " << loadCall.response.loadMessage << "; id: " << load1Id);

  rr.call<temoto_resource_registrar::LoadComponent>("AgentRR", "resourceServer", loadCall, NULL, statusCallback);

  load1Id = loadCall.response.TemotoMetadata.requestId;

  ROS_INFO_STREAM("OUTPUT RESULT: " << loadCall.response.loadMessage << "; id: " << load1Id);

  //bool unloadRes = rr.unload("AgentRR", load1Id);
  //ROS_INFO_STREAM("Unload result: " << unloadRes);


  temoto_resource_registrar::LoadComponent loadCall2;
  loadCall2.request.loadTarget = "TimeService";

  rr.call<temoto_resource_registrar::LoadComponent>("AgentRR", "resourceServer", loadCall2);

  std::string load2Id = loadCall2.response.TemotoMetadata.requestId;

  ROS_INFO_STREAM("OUTPUT RESULT: " << loadCall2.response.loadMessage << "; id: " << load2Id);

  rr.unload("AgentRR", load2Id);

  rr.printCatalog();
*/
  //ros::spin();
  ros::waitForShutdown();

  ROS_INFO("Exiting consumer....");
}