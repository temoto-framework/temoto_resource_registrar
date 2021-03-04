#include "ros/console.h"
#include "ros/ros.h"

#include "temoto_resource_registrar/rr_base.h"

#include "rr/ros1_client.h"
#include "ros1_client.cpp"

#include "rr/ros1_server.h"
#include "ros1_server.cpp"

namespace temoto_resource_registrar
{
  class ResourceRegistrarRos1 : public RrBase<RrServerBase, RrClientBase>
  {
  public:
    ResourceRegistrarRos1(const std::string &name) : RrBase<RrServerBase, RrClientBase>(name) {}

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
      return false;
    }

  protected:
  private:
  };
}