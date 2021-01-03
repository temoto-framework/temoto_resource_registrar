//#include "ResourceRegistrar.cpp"
#include "RRServerBased.cpp"

#include <actionlib_tutorials/Resource1Action.h>
#include <actionlib_tutorials/Resource2Action.h>

/*
void consumerCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("The time is: [%s]", msg->data.c_str());
}
*/

int main(int argc, char **argv)
{
  std::string name = "timeRequestor";
  std::string target = "timePublisher";

  ros::init(argc, argv, name);

  ROS_INFO("init rr");
  RosRR rr(name);

  ROS_INFO("init Request");
  temoto_resource_registrar::RrQueryRequest request("time", "stringServer");

  actionlib_tutorials::Resource1Goal goal;

  goal.message = "test goal";

  RosQuery<actionlib_tutorials::Resource1Action, actionlib_tutorials::Resource1Goal>
      query(request, goal);

  ROS_INFO("calling");
  rr.call(query, 5);

  /*
  std::string name = "timeRequestor";
  std::string target = "timePublisher";

  ros::init(argc, argv, name);

  //RosRR<std_msgs::String> rr(name);

  RRActionManager rr_am_(name);

  temoto_resource_registrar::RrQueryRequest req("give time plz");
  temoto_resource_registrar::RrQueryBase query(target, req);

  ros::Rate loop_rate(2);

  ROS_INFO("Calling");

  rr_am_.call(query);

  ROS_INFO("Call done!");

  ros::NodeHandle n;

  ROS_INFO("Subscribing done!");
  ros::Subscriber sub = n.subscribe(query.response().response_, 100, consumerCallback);

  ros::spin();

  std::cout << query.response().response_ << std::endl;

  //ros::spin();
  */
}