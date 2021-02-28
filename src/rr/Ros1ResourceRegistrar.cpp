#include "ros/console.h"
#include "ros/ros.h"

#include "temoto_resource_registrar/rr_base.h"

#include "Ros1Client.cpp"
#include "Ros1Server.cpp"

template <class ServerType, class ClientType>
class Ros1ResourceRegistrar : public temoto_resource_registrar::RrBase<ServerType, ClientType>
{
public:
  Ros1ResourceRegistrar(const std::string &name) : temoto_resource_registrar::RrBase<ServerType, ClientType>(name) {}

  template <class ClientClass, class ServiceClass>
  void call(const std::string &rr, const std::string &server, ServiceClass &query)
  {
    ROS_INFO_STREAM("CALL ME BABY " << rr << " - " << server);
    std::string clientName = rr + "_" + server;

    if (!this->clients_.exists(clientName))
    {
      ROS_INFO_STREAM("creating client...");

      std::unique_ptr<ClientClass> client = std::make_unique<ClientClass>(clientName);
      ROS_INFO_STREAM("setting catalog");
      client->setCatalog(this->rr_catalog_);
      ROS_INFO_STREAM("adding to clients");
      this->clients_.add(std::move(client));
    }

    auto &clientRef = this->clients_.getElement(clientName);

    auto dynamicRef = dynamic_cast<const ClientClass &>(clientRef);

    ROS_INFO_STREAM("time to query...");

    dynamicRef.invoke(query);
  }

protected:
private:
};
