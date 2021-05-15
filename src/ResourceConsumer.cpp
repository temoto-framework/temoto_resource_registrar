#include "rr/ros1_resource_registrar.h"

#include "temoto_resource_registrar/LoadComponent.h"

std::string rrName = "ConsumerRR";
temoto_resource_registrar::ResourceRegistrarRos1 rr(rrName);

int counter = 0;
int shutdownCounter = 5;
std::string loadId = "";

void statusCallbackAlternate(temoto_resource_registrar::LoadComponent msg, temoto_resource_registrar::Status status)
{
  ROS_INFO_STREAM("-----------------------------------CALLBACK 2" << __func__ << " - " << status.serialised_request_.size() << " - " << msg.request.temoto_metadata.origin_rr);

  counter++;

  if (counter == shutdownCounter)
  {

    rr.printCatalog();
    auto r = rr.getRosChildQueries<temoto_resource_registrar::LoadComponent>(loadId, "resourceServer");

    for (const auto &el : r)
    {
      ROS_INFO_STREAM("id: " << el.first << " - msg: " << el.second.request);
    }

    ROS_INFO_STREAM("getServerRrQueries size: " << r.size());

    ROS_INFO_STREAM("Counter reached, unloading: " << loadId << " from: "
                                                   << "AgentRR");
    bool res = rr.unload("AgentRR", loadId);
    ROS_INFO_STREAM("Unload result: " << res);
  }
}

void statusCallback(temoto_resource_registrar::LoadComponent msg, temoto_resource_registrar::Status status)
{
  ROS_INFO_STREAM("-----------------------------------IN " << __func__ << " - " << status.serialised_request_.size() << " - " << msg.request.temoto_metadata.origin_rr);

  counter++;

  if (counter == 3) {
    rr.registerClientCallback<temoto_resource_registrar::LoadComponent>("AgentRR", "resourceServer", loadId, statusCallbackAlternate);
  }

  if (counter == shutdownCounter)
  {

    rr.printCatalog();
    auto r = rr.getRosChildQueries<temoto_resource_registrar::LoadComponent>(loadId, "resourceServer");

    for (const auto &el : r)
    {
      ROS_INFO_STREAM("id: " << el.first << " - msg: " << el.second.request);
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
  loadCall.request.load_target = "CounterService";
  loadCall.request.Temoto_metadata.request_id = "request_id";
  loadCall.request.Temoto_metadata.servingRr = "servingRr";
  loadCall.request.Temoto_metadata.originRr = "originRr";
  loadCall.request.Temoto_metadata.dependencies = dependencies;
  loadCall.response.load_message = "OK";

  ROS_INFO_STREAM("loadCall.request.load_target: " << loadCall.request.load_target);
  ROS_INFO_STREAM("loadCall.request.Temoto_metadata.request_id: " << loadCall.request.Temoto_metadata.request_id);
  ROS_INFO_STREAM("loadCall.request.Temoto_metadata.servingRr: " << loadCall.request.Temoto_metadata.servingRr);
  ROS_INFO_STREAM("loadCall.request.Temoto_metadata.originRr: " << loadCall.request.Temoto_metadata.originRr);
  ROS_INFO_STREAM("loadCall.response.load_message: " << loadCall.response.load_message);

  ROS_INFO_STREAM("dependency size: " << loadCall.request.Temoto_metadata.dependencies.size());
  for (const auto &el : loadCall.request.Temoto_metadata.dependencies)
  {
    ROS_INFO_STREAM("dependency: " << el);
  }

  ROS_INFO_STREAM("------------------------------------------------------------");

  Ros1Query<temoto_resource_registrar::LoadComponent> rosQuery(loadCall);

  rosQuery.request().Temoto_metadata.request_id = "modified";

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

  ROS_INFO_STREAM("loadCall.request.load_target: " << newLoadCall.request.load_target);
  ROS_INFO_STREAM("loadCall.request.Temoto_metadata.request_id: " << newLoadCall.request.Temoto_metadata.request_id);
  ROS_INFO_STREAM("loadCall.request.Temoto_metadata.servingRr: " << newLoadCall.request.Temoto_metadata.servingRr);
  ROS_INFO_STREAM("loadCall.request.Temoto_metadata.originRr: " << newLoadCall.request.Temoto_metadata.originRr);
  ROS_INFO_STREAM("loadCall.response.load_message: " << newLoadCall.response.load_message);

  ROS_INFO_STREAM("dependency size: " << newLoadCall.request.Temoto_metadata.dependencies.size());
  for (const auto &el : newLoadCall.request.Temoto_metadata.dependencies)
  {
    ROS_INFO_STREAM("dependency: " << el);
  }*/

  temoto_resource_registrar::LoadComponent loadCall;
  loadCall.request.load_target = "CounterService";

  ROS_INFO_STREAM("Calling server with error expectation...");
  try
  {
    rr.call<temoto_resource_registrar::LoadComponent>("AgentRR", "resourceServer", loadCall, statusCallback);

    loadId = loadCall.response.temoto_metadata.request_id;

    
  }
  catch (const resource_registrar::TemotoErrorStack &e)
  {
    ROS_WARN_STREAM("Something bad happened. What is the cause i wonder? : " << e.getMessage());
  }

  /*ROS_INFO_STREAM("OUTPUT RESULT: " << loadCall.response.load_message << "; id: " << load1Id);

  rr.call<temoto_resource_registrar::LoadComponent>("AgentRR", "resourceServer", loadCall, NULL, statusCallback);

  load1Id = loadCall.response.Temoto_metadata.request_id;

  ROS_INFO_STREAM("OUTPUT RESULT: " << loadCall.response.load_message << "; id: " << load1Id);

  //bool unloadRes = rr.unload("AgentRR", load1Id);
  //ROS_INFO_STREAM("Unload result: " << unloadRes);


  temoto_resource_registrar::LoadComponent loadCall2;
  loadCall2.request.load_target = "TimeService";

  rr.call<temoto_resource_registrar::LoadComponent>("AgentRR", "resourceServer", loadCall2);

  std::string load2Id = loadCall2.response.Temoto_metadata.request_id;

  ROS_INFO_STREAM("OUTPUT RESULT: " << loadCall2.response.load_message << "; id: " << load2Id);

  rr.unload("AgentRR", load2Id);

  rr.printCatalog();
*/
  //ros::spin();
  ros::waitForShutdown();

  ROS_INFO("Exiting consumer....");
}