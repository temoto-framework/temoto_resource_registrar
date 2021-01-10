#include "RRServerBased.cpp"

#include "std_msgs/String.h"
#include "temoto_resource_registrar/rr_server_base.h"

#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <actionlib_tutorials/Resource1Action.h>
#include <actionlib_tutorials/Resource2Action.h>

/*
bool initialized = false;

static std::string pubName = "timePublisher";

std::string getTime()
{
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
  return oss.str();
}

void runLoop()
{
  ros::NodeHandle n;
  ros::Publisher timePublisher = n.advertise<std_msgs::String>(pubName, 100);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    if (initialized)
    {

      time_t rawtime;

      std_msgs::String msg;
      msg.data = getTime();

      std::cout << "spinning..." << msg.data << std::endl;

      timePublisher.publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
*/

void startCB(temoto_resource_registrar::RrQueryBase *query)
{
  std::cout << "start CB" << std::endl;

  //std::move(query);

  temoto_resource_registrar::RrQueryResponse resp("Starting up!");

  query->updateResponse(resp);

  //std::cout << query.goal_ << std::endl;

  std::cout << query->response().response_ << std::endl;
  std::cout << "start CB end" << std::endl;
};

void stopCB(temoto_resource_registrar::RrQueryBase *query)
{
  std::cout << "stop CB" << std::endl;
};

int main(int argc, char **argv)
{
  std::string name = "timePublisher";
  ros::init(argc, argv, name);

  RosRR rr("timeRR");

  auto server = std::make_unique<ActionBasedServer<actionlib_tutorials::Resource1Action,
                                                   actionlib_tutorials::Resource1Feedback,
                                                   actionlib_tutorials::Resource1Result,
                                                   actionlib_tutorials::Resource1GoalConstPtr>>("stringServer", &startCB, &stopCB);

  rr.registerServer(std::move(server));

  auto server2 = std::make_unique<ActionBasedServer<actionlib_tutorials::Resource2Action,
                                                    actionlib_tutorials::Resource2Feedback,
                                                    actionlib_tutorials::Resource2Result,
                                                    actionlib_tutorials::Resource2GoalConstPtr>>("intServer", &startCB, &stopCB);

  rr.registerServer(std::move(server2));

  /*RRActionManager rr_am_(name);

  rr_am_.registerServer(std::make_unique<ServerDerivate<std_msgs::String>>(name, &startCB, &stopCB));

  runLoop();

  ros::spin();*/

  ros::spin();
}