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
      : rr_registry_(std::make_shared<RrRegistry>()), rr_message_registry_(std::make_shared<RrMessageRegistry>()), name_(name){};

  void RrBase::addServer(std::unique_ptr<RrServerBase> baseServer)
  {
    rr_registry_->addServer(std::move(baseServer));
  };

  void RrBase::addClient(std::unique_ptr<RrClientBase> baseClient){};

  bool RrBase::exists(std::string serverId)
  {
    return rr_registry_->hasServer(serverId);
  };

  // rrBase vajab funktsiooni RrQueryBase ära handliks

  //handleQuery(RrQueryBase &resource)
  /**
   * serveri leidmine
   * query unikaalsuse kontroll, vaadatakse kas teha load või mitte
   * skip esialgu - binditakse status callback
   * KUI unikaalne:
   *  jooksutatakse load callback
   *  vormistatakse response, et misiganes response-i tuleb
   *  talletakse rr_registry
   *  void, RrQueryBase sees on response
   * ELSE
   *  rr_registry võa väärtus ja tehtud
   */

  void RrBase::call(RrQueryBase &resource, RrBase &rr)
  {

    if (rr.exists(resource.target()))
    {
      rr.call(resource);
      /*if (rr_message_registry_->hasResponse(resource))
      {
        LOG(INFO) << "has response";
        resource.updateResponse(rr_message_registry_->response(resource));
      }
      else
      {
        LOG(INFO) << "No response found. Fetching it";
        rr.fetchServer(resource.target())->loadResource();
        resource.updateResponse(rr.fetchServer(resource.target())->processRequest(resource.request()));
      }

      rr_message_registry_*/
    }
    else
    {
      LOG(INFO) << "nothing to do";
    }
  };

  void RrBase::call(RrQueryBase &resource)
  {
    if (rr_message_registry_->hasResponse(resource))
    {
      LOG(INFO) << "has response";
    }
    else
    {
      LOG(INFO) << "No response found. Fetching it";
      RrServerBase *server = rr_registry_->fetchServer(resource.target());
      server->loadResource();
      resource.updateResponse(server->processRequest(resource.request()));
    }
    //enriching response
    rr_message_registry_->response(resource);
  };

  RrServerBase *RrBase::fetchServer(std::string serverId)
  {
    return rr_registry_->fetchServer(serverId);
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
