#include "ros/console.h"
#include "ros/ros.h"

#include "temoto_resource_registrar/rr_base.h"

#include "temoto_resource_registrar/UnloadComponent.h"

#include "rr/ros1_client.h"
#include "rr/ros1_server.h"

namespace temoto_resource_registrar
{
  class ResourceRegistrarRos1 : public RrBase<RrServerBase, RrClientBase>
  {
  public:
    ResourceRegistrarRos1(const std::string &name) : RrBase<RrServerBase, RrClientBase>(name)
    {
      startUnloadService();
    }

    template <class ServiceClass>
    void call(const std::string &rr, const std::string &server, ServiceClass &query)
    {
      ROS_INFO_STREAM("CALL ME BABY " << rr << " - " << server);
      std::string clientName = rr + "_" + server;

      if (!this->clients_.exists(clientName))
      {
        ROS_INFO_STREAM("creating client...");

        auto client = std::make_unique<Ros1Client<ServiceClass>>(clientName);
        ROS_INFO_STREAM("setting catalog");
        client->setCatalog(this->rr_catalog_);
        ROS_INFO_STREAM("adding to clients");
        this->clients_.add(std::move(client));
      }

      auto &clientRef = this->clients_.getElement(clientName);

      auto dynamicRef = dynamic_cast<const Ros1Client<ServiceClass> &>(clientRef);

      ROS_INFO_STREAM("time to query...");

      dynamicRef.invoke(query);
    }

    bool unload(const std::string &rr, const std::string &id)
    {
      ROS_INFO_STREAM("unload Called");

      std::string clientName = rr + "_unloader";

      if (unload_clients_.count(clientName) == 0)
      {
        ROS_INFO_STREAM("creating unload client...");
        auto sc = nh_.serviceClient<UnloadComponent>(clientName);
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
    ros::NodeHandle nh_;
    ros::ServiceServer service_;

    void startUnloadService()
    {
      service_ = nh_.advertiseService(name() + "_unloader", &ResourceRegistrarRos1::unloadCallback, this);
    }

    bool unloadCallback(UnloadComponent::Request &req, UnloadComponent::Response &res)
    {
      std::string id = req.target;
      res.status = localUnload(id);
      return true;
    }
  };
}