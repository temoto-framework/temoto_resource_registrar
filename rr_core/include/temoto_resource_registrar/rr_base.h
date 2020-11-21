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

#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_BASE_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_BASE_H

#include "rr_client_base.h"
#include "rr_query_base.h"
#include "rr_registry.h"
#include "rr_resource.h"
#include "rr_server_base.h"
#include <iostream>
#include <map>

namespace temoto_resource_registrar
{
  class RrBase
  {
  public:
    RrBase(std::string name)
        : rr_registry_(std::make_shared<RrRegistry>()), name_(name){};

    void addServer(std::unique_ptr<RrServerBase> base_server)
    {
      rr_servers_.insert({base_server->id(), std::move(base_server)});
    };

    void addClient(std::unique_ptr<RrClientBase> baseClient){};

    bool exists(std::string serverId)
    {
      return rr_servers_.count(serverId) > 0;
    };

    void call(RrQueryBase &resource, RrBase &base)
    {

      std::cout << typeid(resource).name() << " - " << resource.target() << std::endl;

      if (base.exists(resource.target()))
      {
        LOG(INFO) << "Server with ID '" << resource.target() << "' found! "
                  << "Found in: " << base.id();
      }
      else
      {
        LOG(INFO) << "Server with ID '" << resource.target() << "' NOT found!"
                  << " searched in: " << base.id();
      }
    };

    void print()
    {
      for (const auto &server : rr_servers_)
      {
        server.second->print();
      }
    };

    std::string id()
    {
      return name_;
    }

  private:
    std::map<std::string, std::unique_ptr<RrServerBase>> rr_servers_;
    std::map<std::string, std::unique_ptr<RrClientBase>> rr_clients_;
    RrRegistryPtr rr_registry_;
    std::string name_;
  };

} // namespace temoto_resource_registrar

#endif