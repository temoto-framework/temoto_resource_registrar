#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include "temoto_resource_registrar/rr_base.h"
#include "temoto_resource_registrar/rr_query_base.h"
#include "temoto_resource_registrar/rr_server_base.h"

template <class requestClass, class feedbackClass, class responseClass, class goalClass>
class ActionBasedServer : public temoto_resource_registrar::RrServerBase
{
public:
  ActionBasedServer(const std::string &name, void (*loadCallback)(), void (*unLoadCallback)())
      : RrServerBase(name, __func__, loadCallback, unLoadCallback),
        as_(nh_, name, boost::bind(&ActionBasedServer::serveCallback, this, _1), false)
  {
    std::cout << "Starting AS " << name_ << std::endl;
    as_.start();
  }

  void serveCallback(const goalClass &goal)
  {
    std::cout << "serveCallback" << std::endl;
    loadResource();
  }

  std::string id()
  {
    return name_;
  }

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<requestClass> as_;
  feedbackClass feedback_;
  responseClass result_;

private:
};

template <class requestClass, class goalClass>
class RosQuery : public temoto_resource_registrar::RrQueryBase
{
public:
  RosQuery(temoto_resource_registrar::RrQueryRequest &request, const goalClass &goal) : RrQueryBase(request), goal_(goal) {}
  typedef requestClass req_class_;
  typedef goalClass goal_class_;
  goalClass goal_;

protected:
private:
};

class RosRR
{
public:
  RosRR(const std::string &name) : rr_(name) {}

  void registerServer(std::unique_ptr<temoto_resource_registrar::RrServerBase> server)
  {
    std::cout << server->id() << std::endl;
    ROS_INFO("Registered server !");
    rr_.addServer(std::move(server));
  }

  template <class action, class goalClass>
  void call(RosQuery<action, goalClass> &query, float timeout)
  {
    ROS_INFO("Calling...");
    actionlib::SimpleActionClient<action> ac(query.request().target_, true);
    ac.waitForServer();

    ac.sendGoal(query.goal_);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
      ROS_INFO("Action did not finish before the time out.");
    }
  }

protected:
  temoto_resource_registrar::RrBase rr_;

private:
};

/*
void startCB()
{
  std::cout << "start CB" << std::endl;
};

static void stopCB()
{
  std::cout << "stop CB" << std::endl;
};*/

/*
int main(int argc, char **argv)
{
  std::cout << "ros init" << std::endl;

  ros::init(argc, argv, "Resource1Node");

  std::cout << "init server" << std::endl;

  ActionBasedServer<actionlib_tutorials::Resource1Action,
                    actionlib_tutorials::Resource1Feedback,
                    actionlib_tutorials::Resource1Result,
                    actionlib_tutorials::Resource1GoalConstPtr>
      as("Resource1", &startCB, &stopCB);

  std::cout << "lol" << std::endl;

  ros::spin();
}*/