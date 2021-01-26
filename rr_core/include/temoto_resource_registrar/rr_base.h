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

#include "rr_catalog.h"
#include <iostream>
#include <unordered_map>

namespace temoto_resource_registrar
{
  const static std::string CLIENT_SUFIX = ";CLIENT";

  template <class ContentClass>
  class MapContainer
  {
  public:
    bool add(std::unique_ptr<ContentClass> content)
    {
      auto ret = rr_contents_.insert(std::make_pair(content->id(), std::move(content)));
      return ret.second;
    };

    bool remove(const std::string &id)
    {
      return rr_contents_.erase(id) > 0;
    };

    bool exists(const std::string &id)
    {
      auto it = rr_contents_.find(id);
      if (it != rr_contents_.end())
      {
        return true;
      }
      return false;
    };

    std::vector<std::string> getIds()
    {
      std::vector<std::string> ids;
      for (auto it = rr_contents_.begin(); it != rr_contents_.end(); ++it)
      {
        ids.push_back(it->first);
      }
      return ids;
    };

    ContentClass *getElement(std::string key)
    {
      auto it = rr_contents_.find(key);
      if (it != rr_contents_.end())
      {
        return it->second.get();
      }
    };

  protected:
    std::unordered_map<std::string, std::unique_ptr<ContentClass>>
        rr_contents_;
  };

  template <class serverClass>
  class RrServers : public MapContainer<serverClass>
  {
  };

  template <class clientClass>
  class RrClients : public MapContainer<clientClass>
  {
  };

  template <class ServerType, class ClientType>
  class RrBase
  {
  public:
    RrBase(std::string name) : name_(name),
                               rr_catalog_(std::make_shared<RrCatalog>()){};

    template <class CallClientClass>
    void call(std::string rr, std::string server, const RrQueryBase &query)
    {
      std::string clientName = rr + ";" + server + CLIENT_SUFIX;
      if (!clients_.exists(clientName))
      {
        std::cout << "creating client! " << clientName << std::endl;

        std::unique_ptr<CallClientClass> client = std::make_unique<CallClientClass>(clientName);
        client->setCatalog(rr_catalog_);

        clients_.add(std::move(client));
      }

      auto client = dynamic_cast<CallClientClass *>(clients_.getElement(clientName));

      client->invoke(query);

      // add client back to pool
      //clients_.add(std::move(dynamic_cast<ClientType *>(client)));
    };

    template <class CallServerClass>
    void call(RrBase &target, std::string server, RrQueryBase *query)
    {
      target.callServer<CallServerClass>(server, query);
    };

    template <class CallServerClass>
    void callServer(std::string server, RrQueryBase *query)
    {
      auto serverPtr = dynamic_cast<CallServerClass *>(servers_.getElement(server));

      std::cout << "calli server CB!" << std::endl;

      serverPtr->processQuery(query);
    };

    const std::string id();

    size_t serverCount()
    {
      return servers_.getIds().size();
    };

    //bool hasResponse(RrQueryBase &resource);
    //void registerResponse(RrQueryBase &resource);

    void registerServer(std::unique_ptr<ServerType> serverPtr)
    {
      serverPtr->setCatalog(rr_catalog_);
      servers_.add(std::move(serverPtr));
    };

  private:
    RrCatalogPtr rr_catalog_;
    std::string name_;

    RrServers<ServerType> servers_;
    RrClients<ClientType> clients_;
  }; // namespace temoto_resource_registrar

} // namespace temoto_resource_registrar

#endif