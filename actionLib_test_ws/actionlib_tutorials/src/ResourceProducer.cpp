#include "ResourceRegistrar.cpp"

#include "std_msgs/String.h"
#include "temoto_resource_registrar/rr_server_base.h"

#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

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

void startCB()
{
  std::cout << "start CB" << std::endl;
  initialized = true;
};

static void stopCB()
{
  std::cout << "stop CB" << std::endl;
  initialized = false;
};

int main(int argc, char **argv)
{
  std::string name = "timePublisher";
  ros::init(argc, argv, name);

  //RosRR<std_msgs::String> rr(name);

  RRActionManager rr_am_(name);

  rr_am_.registerServer(std::make_unique<ServerDerivate<std_msgs::String>>(name, &startCB, &stopCB));

  runLoop();

  ros::spin();
}