#include "ros/console.h"
#include "ros/ros.h"

#include "temoto_resource_registrar/rr_base.h"

#include "temoto_resource_registrar/UnloadComponent.h"

#include "rr/ros1_client.h"
#include "rr/ros1_server.h"

namespace temoto_resource_registrar
{
  class ResourceRegistrarRos1 : public RrBase
  {
  public:
    ResourceRegistrarRos1(const std::string &name) : RrBase(name)
    {}

    void init() {
      startUnloadService();
    }

    template <class QueryType>
    void call(const std::string &rr,
              const std::string &server,
              QueryType &query,
              RrQueryBase *parentQuery = NULL,
              StatusFunction statusFunc = NULL,
              bool overrideStatus = false)
    {
      ROS_INFO_STREAM("Master call from ros1 impl");

      Ros1Query<typename QueryType::Request, typename QueryType::Response> wrappedQuery(query.request, query.response);

      privateCall<Ros1Client<QueryType>,
                  Ros1Server<QueryType>,
                  Ros1Query<typename QueryType::Request,
                            typename QueryType::Response>>(&rr,
                                                           NULL,
                                                           server,
                                                           wrappedQuery,
                                                           parentQuery,
                                                           statusFunc,
                                                           overrideStatus);

      QueryType sc;
      sc.request = wrappedQuery.request();
      sc.response = wrappedQuery.response();
      query = sc;
    }

    bool unload(const std::string &rr, const std::string &id)
    {
      ROS_INFO_STREAM("unload Called");

      ros::NodeHandle nh;

      std::string clientName = rr + "_unloader";

      if (unload_clients_.count(clientName) == 0)
      {
        ROS_INFO_STREAM("creating unload client...");
        auto sc = nh.serviceClient<UnloadComponent>(clientName);
        auto client = std::make_unique<ros::ServiceClient>(sc);
        unload_clients_[clientName] = std::move(client);
      }

      temoto_resource_registrar::UnloadComponent unloadSrv;
      unloadSrv.request.target = id;

      ROS_INFO_STREAM("executing unload call");

      bool res = unload_clients_[clientName]->call(unloadSrv);

      ROS_INFO_STREAM("executing unload call done " << res);

      return res;
    }

  protected:
    std::unordered_map<std::string, std::unique_ptr<ros::ServiceClient>> unload_clients_;

  private:
    
    ros::ServiceServer service_;

    

    void startUnloadService()
    {
      ros::NodeHandle nh;
      service_ = nh.advertiseService(name() + "_unloader", &ResourceRegistrarRos1::unloadCallback, this);
    }

    bool unloadCallback(UnloadComponent::Request &req, UnloadComponent::Response &res)
    {
      std::string id = req.target;
      res.status = localUnload(id);
      return true;
    }
  };
}