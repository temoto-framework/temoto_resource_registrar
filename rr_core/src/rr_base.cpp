#include "temoto_resource_registrar/rr_base.h"

#include <iostream>

namespace temoto_resource_registrar
{

  RrBase::RrBase(std::string name)
      : rr_registry_(std::make_shared<RrRegistry>()), name_(name){};

  void RrBase::addServer(std::unique_ptr<RrServerBase> base_server)
  {
    rr_registry_->addServer(std::move(base_server));
  };

  void RrBase::addClient(std::unique_ptr<RrClientBase> baseClient){};

  bool RrBase::exists(std::string serverId)
  {
    return rr_registry_->hasServer(serverId);
  };

  void RrBase::call(RrQueryBase &resource, RrBase &rr)
  {
    if (rr.exists(resource.target()))
    {
      RrServerBase *server = rr.fetchServer(resource.target());
      server->loadResource();
    }
  };

  RrServerBase *RrBase::fetchServer(std::string serverId)
  {
    return fetchServer(serverId);
  };

  void RrBase::print()
  {
    for (const auto &server : rr_registry_->registeredServers())
    {
      rr_registry_->fetchServer(server)->print();
    }
  };

  const std::string RrBase::id()
  {
    return name_;
  };

} // namespace temoto_resource_registrar
