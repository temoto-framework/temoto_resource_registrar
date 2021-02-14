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

#include <iostream>
#include <thread>
#include <unordered_map>

#include "rr_catalog.h"
#include "rr_client_base.h"

#include <mutex>

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

    const ContentClass &getElement(std::string key)
    {
      auto it = rr_contents_.find(key);
      if (it != rr_contents_.end())
      {
        return *(it->second.get());
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
    void call(const std::string &rr, const std::string &server, RrQueryBase &query)
    {
      call<CallClientClass>(&rr, NULL, server, query);
    }

    template <class ServType, class QueryType>
    void call(RrBase &target, const std::string &server, QueryType &query)
    {
      call<RrClientBase, ServType, QueryType>(NULL, &(target), server, query);
    }

    const std::string id();

    size_t serverCount()
    {
      return servers_.getIds().size();
    }

    void registerServer(std::unique_ptr<ServerType> serverPtr)
    {
      serverPtr->setCatalog(rr_catalog_);
      servers_.add(std::move(serverPtr));
    }

    template <class ServType, class QueryType>
    void handleInternalCall(const std::string &server, QueryType &query)
    {
      auto &serverRef = servers_.getElement(server);

      auto dynamicRef = dynamic_cast<const ServType &>(serverRef);

      dynamicRef.processQuery(query);
    }

    template <class ServType>
    bool unload(RrBase &target, const std::string &id)
    {
      return target.unload<ServType>(id);
    }

    template <class ServType>
    bool unload(const std::string &id)
    {

      std::string serverId = rr_catalog_->getIdServer(id);

      auto &serverRef = servers_.getElement(serverId);
      auto dynamicRef = dynamic_cast<const ServType &>(serverRef);
      return dynamicRef.unloadMessage(id);
    }

    void printCatalog()
    {
      rr_catalog_->print();
    }

  private:
    RrCatalogPtr rr_catalog_;
    std::string name_;

    RrServers<ServerType> servers_;
    RrClients<ClientType> clients_;

    std::thread::id workId;
    std::mutex mtx;

    bool processing = false;

    template <class CallClientClass, class ServType, class QueryType>
    void call(const std::string *rr, RrBase *target, const std::string &server, QueryType &query)
    {
      mtx.lock();

      if (processing == true)
      {
        std::cout << "need to record dependencies " << name_ << std::endl;
      }
      else
      {
        processing = true;
        std::cout << "starting processing " << name_ << std::endl;
      }

      std::cout << "---------------call in " << name_ << std::endl;
      std::cout << "---------------call id? " << name_ << " - " << query.id() << std::endl;

      workId = std::this_thread::get_id();

      // In case we have a client call, not a internal call
      if ((rr == NULL) && !(target != NULL))
      {
        handleClientCall<CallClientClass>(*(rr), server, query);
      }
      else
      {
        target->handleInternalCall<ServType, QueryType>(server, query);
      }

      std::cout << "---------------call id end? " << name_ << " - " << query.id() << std::endl;
      processing = false;
      std::cout << "ending processing " << name_ << std::endl;
      mtx.unlock();
    }

    template <class CallClientClass>
    void handleClientCall(const std::string &rr, const std::string &server, RrQueryBase &query)
    {
      std::string clientName = rr + ";" + server + CLIENT_SUFIX;

      if (!clients_.exists(clientName))
      {
        std::cout << "creating client! " << clientName << std::endl;

        std::unique_ptr<CallClientClass> client = std::make_unique<CallClientClass>(clientName);
        client->setCatalog(rr_catalog_);

        clients_.add(std::move(client));
      }

      auto &client = clients_.getElement(clientName);

      client.invoke(query);
    }
  };

} // namespace temoto_resource_registrar

#endif