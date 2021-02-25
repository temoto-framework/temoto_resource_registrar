#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/Resource1Action.h>
#include <actionlib_tutorials/Resource2Action.h>

#include <temoto_resource_registrar/rr_base.h>
#include <temoto_resource_registrar/rr_query_base.h>
#include <temoto_resource_registrar/rr_resource.h>
#include <temoto_resource_registrar/rr_server_base.h>

template <class serverClass>
class ServerDerivate : public temoto_resource_registrar::RrServerBase
{
public:
  ServerDerivate(const std::string &name, void (*loadCallback)(), void (*unLoadCallback)())
      : RrServerBase(name, __func__, loadCallback, unLoadCallback)
  {
  }

  void print()
  {
  }
};

{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::Resource1Action> as_;

public:
private:
};

int main(int argc, char **argv)
{
  temoto_resource_registrar::RrBase rr_m0 = temoto_resource_registrar::RrBase("rr_m0");

  return 0;
}
