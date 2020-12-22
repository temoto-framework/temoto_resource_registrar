#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/RequestResourceAction.h>

#include "temoto_resource_registrar/rr_base.h"
#include "temoto_resource_registrar/rr_query_base.h"
#include "temoto_resource_registrar/rr_server_base.h"

template <class serverClass>
class ServerDerivate : public temoto_resource_registrar::RrServerBase
{
public:
  ServerDerivate(const std::string &name, void (*loadCallback)(), void (*unLoadCallback)())
      : RrServerBase(name, __func__, loadCallback, unLoadCallback) {}
};

class RRActionManager
{
protected:
  temoto_resource_registrar::RrBase rr_;
  std::string rr_name_;
  std::string rr_as_name_;
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::RequestResourceAction> as_;
  actionlib_tutorials::RequestResourceFeedback feedback_;
  actionlib_tutorials::RequestResourceResult result_;

public:
  RRActionManager(const std::string &name) : rr_(name),
                                             rr_name_(name),
                                             rr_as_name_(name + "_as"),
                                             as_(nh_, rr_as_name_, boost::bind(&RRActionManager::executeCB, this, _1), false)
  {
    std::cout << "Starting AS " << rr_as_name_ << std::endl;
    as_.start();
  }

  void executeCB(const actionlib_tutorials::RequestResourceGoalConstPtr &goal)
  {
    std::cout << "executeCB" << std::endl;
    ROS_INFO("%s: Succeeded", rr_as_name_.c_str());
    ROS_INFO("%s: MSG", goal->message.c_str());

    ROS_INFO("Creating copy of request on server side");
    temoto_resource_registrar::RrQueryRequest req(goal->message);
    temoto_resource_registrar::RrQueryBase query(rr_name_, req);

    rr_.call(query);

    result_.result = query.response().response_;
    ROS_INFO("Sending result: %s", result_.result.c_str());
    as_.setSucceeded(result_);
  };

  void registerServer(std::unique_ptr<temoto_resource_registrar::RrServerBase> server)
  {
    ROS_INFO("Registered server!");
    rr_.addServer(std::move(server));
  }

  void call(temoto_resource_registrar::RrQueryBase &query)
  {
    ROS_INFO("Calling...");
    if (!rr_.hasResponse(query))
    {
      ROS_INFO("new response... creating client");
      std::string queryTarget = query.target() + "_as";
      ROS_INFO("Sending request to: %s", queryTarget.c_str());
      actionlib::SimpleActionClient<actionlib_tutorials::RequestResourceAction> ac_(queryTarget, true);

      ROS_INFO("Waiting for action server to start.");
      ac_.waitForServer();
      ROS_INFO("DONE!");
      actionlib_tutorials::RequestResourceGoal goal;

      goal.message = query.request().message_;

      ac_.sendGoal(goal);

      bool finished_before_timeout = ac_.waitForResult(ros::Duration(50.0));

      if (finished_before_timeout)
      {
        actionlib::SimpleClientGoalState state = ac_.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        std::cout << ac_.getResult()->result << std::endl;
        query.updateResponse(temoto_resource_registrar::RrQueryResponse(ac_.getResult()->result));
      }
    }
    ROS_INFO("registerResponse");
    rr_.registerResponse(query);
  }

private:
};