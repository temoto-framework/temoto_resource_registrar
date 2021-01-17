/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2020 TeMoto Telerobotics
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "temoto_resource_registrar/rr_base.h"

#include <iostream>

namespace temoto_resource_registrar
{

  RrBase::RrBase(std::string name)
      : rr_registry_(std::make_shared<RrConnectionRegistry>()),
        rr_message_registry_(std::make_shared<RrCatalog>()),
        name_(name){};

  void RrBase::addServer(std::unique_ptr<RrServerBase> baseServer)
  {
    baseServer->setCatalog(rr_message_registry_);
    rr_registry_->addServer(std::move(baseServer));
  }

  void RrBase::addClient(std::unique_ptr<RrClientBase> baseClient){};

  bool RrBase::exists(std::string serverId)
  {
    return rr_registry_->hasServer(serverId);
  }

  void RrBase::call(RrQueryBase &resource, RrBase &rr)
  {
    if (rr.exists(resource.request().target_))
    {
      rr.call(resource);
    }
  }

  bool RrBase::hasResponse(RrQueryBase &resource)
  {
    return rr_message_registry_->hasResponse(resource);
  }

  void RrBase::registerResponse(RrQueryBase &resource)
  {
    rr_message_registry_->respond(resource);
  }

  void RrBase::call(RrQueryBase &resource)
  {
    if (!(rr_message_registry_->hasResponse(resource)))
    {
      RrServerBase *server = rr_registry_->fetchServer(resource.request().target_);
      server->processQuery(&resource);
    }
    //enriching response
    rr_message_registry_->respond(resource);
  }

  RrServerBase *RrBase::fetchServer(std::string serverId)
  {
    return rr_registry_->fetchServer(serverId);
  }

  void RrBase::print()
  {
    for (const auto &server : rr_registry_->registeredServers())
    {
      rr_registry_->fetchServer(server)->print();
    }
  }

  const std::string RrBase::id()
  {
    return name_;
  }

  int RrBase::serverCount()
  {
    return rr_registry_->registeredServers().size();
  }

} // namespace temoto_resource_registrar
