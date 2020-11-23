#include "temoto_resource_registrar/rr_base.h"

#include <iostream>

namespace temoto_resource_registrar
{

  class ServerNotFoundException
  {
  public:
    ServerNotFoundException(const std::string &msg) : msg_(msg) {}
    ~ServerNotFoundException() {}
    std::string getMessage() const { return (msg_); }

  private:
    std::string msg_;
  };

  RrBase::RrBase(std::string name)
      : rr_registry_(std::make_shared<RrRegistry>()), name_(name){};

  void RrBase::addServer(std::unique_ptr<RrServerBase> base_server)
  {
    rr_servers_.insert(std::make_pair(base_server->id(), std::move(base_server)));
  };

  void RrBase::addClient(std::unique_ptr<RrClientBase> baseClient){};

  bool RrBase::exists(std::string serverId)
  {
    return rr_servers_.count(serverId) > 0;
  };

  void RrBase::call(RrQueryBase &resource, RrBase &base)
  {
    if (base.exists(resource.target()))
    {
      RrServerBase *server = base.fetchServer(resource.target());
      server->loadResource();
    }
    else
    {
    }
  };

  RrServerBase *RrBase::fetchServer(std::string serverId)
  {
    auto it = rr_servers_.find(serverId);
    if (it != rr_servers_.end())
    {
      return it->second.get();
    }
    throw(ServerNotFoundException("Could not find server: " + serverId));
  }

  void RrBase::print()
  {
    for (const auto &server : rr_servers_)
    {
      server.second->print();
    }
  };

  const std::string RrBase::id()
  {
    return name_;
  }

} // namespace temoto_resource_registrar
