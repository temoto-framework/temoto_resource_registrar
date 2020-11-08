#include "temoto_resource_registrar/rr_base.h"
#include <iostream>

namespace temoto_resource_registrar
{
  RrBase::RrBase()
      : rr_registry_(std::make_shared<RrRegistry>())
  {
  }

  void RrBase::addServer(std::unique_ptr<RrServerBase> base_server)
  {
    rr_servers_.insert({base_server->id(), std::move(base_server)});
  }

  bool RrBase::exists(std::unique_ptr<RrServerBase> server)
  {
    return rr_servers_.count(server->id()) > 0;
  }

  void RrBase::call()
  {
  }

  void RrBase::print()
  {
    for (const auto &server : rr_servers_)
    {
      server.second->print();
    }
  }

} // namespace temoto_resource_registrar
