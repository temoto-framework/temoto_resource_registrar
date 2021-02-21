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
#include "rr_query_base.h"
#include "rr_status.h"

#include <mutex>

namespace temoto_resource_registrar
{
  typedef void (*StatusFunction)(const std::string &id, Status status, std::string &message);

  const static std::string CLIENT_SUFIX = ";CLIENT";

  template <class ContentClass>
  class MapContainer
  {
  public:
    bool add(std::unique_ptr<ContentClass> content)
    {
      auto ret = rr_contents_.insert(std::make_pair(content->id(), std::move(content)));
      return ret.second;
    }

    bool remove(const std::string &id)
    {
      return rr_contents_.erase(id) > 0;
    }

    bool exists(const std::string &id)
    {
      auto it = rr_contents_.find(id);
      if (it != rr_contents_.end())
      {
        return true;
      }
      return false;
    }

    std::vector<std::string> getIds()
    {
      std::vector<std::string> ids;
      for (auto it = rr_contents_.begin(); it != rr_contents_.end(); ++it)
      {
        ids.push_back(it->first);
      }
      return ids;
    }

    const ContentClass &getElement(const std::string &key)
    {
      auto it = rr_contents_.find(key);
      if (it != rr_contents_.end())
      {
        return *(it->second.get());
      }
    }

    const bool unload(const std::string &key, const std::string &id)
    {
      auto it = rr_contents_.find(key);
      if (it != rr_contents_.end())
      {
        return (it->second.get())->unloadMessage(id);
      }
      return false;
    }

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

    const std::string id();

    template <class CallClientClass>
    void call(const std::string &rr, const std::string &server, RrQueryBase &query)
    {
      call<CallClientClass>(&rr, NULL, server, query);
    }

    template <class ServType, class QueryType>
    void call(RrBase &target,
              const std::string &server,
              QueryType &query,
              RrQueryBase *parentQuery = NULL,
              StatusFunction statusFunc = NULL,
              bool overrideStatus = false)
    {
      privateCall<RrClientBase, ServType, QueryType>(NULL, &(target), server, query, parentQuery, statusFunc, overrideStatus);
    }

    size_t serverCount() { return servers_.getIds().size(); }

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

    bool unload(RrBase &target, const std::string &id) { return target.unload(id); }

    bool unload(const std::string &id)
    {

      std::string serverId = rr_catalog_->getIdServer(id);

      auto dependencyMap = rr_catalog_->getDependencies(id);
      if (dependencyMap.size() > 0)
      {
        for (auto const &dependency : dependencyMap)
        {
          std::string dependencyServer = rr_references_[dependency.second]->resolveQueryServerId(dependency.first);
          bool unloadStatus = rr_references_[dependency.second]->unloadByServerAndQuery(dependencyServer, dependency.first);

          if (unloadStatus)
            rr_catalog_->unloadDependency(id, dependency.first);
        }
      }

      return unloadByServerAndQuery(serverId, id);
    }

    void printCatalog() { rr_catalog_->print(); }

    std::string name() { return name_; }

    void setRrReferences(const std::unordered_map<std::string, RrBase *> &references) { rr_references_ = references; }

    std::string resolveQueryServerId(const std::string &id) { return rr_catalog_->getIdServer(id); }

    bool unloadByServerAndQuery(const std::string &server, const std::string &id) { return servers_.unload(server, id); }

    void sendStatus(const std::string &id, Status status, std::string &message)
    {
      std::unordered_map<std::string, std::string> notifyIds = rr_catalog_->getAllQueryIds(id);

      for (auto const &notId : notifyIds)
      {
        std::cout << notId.first << " - " << notId.second << std::endl;
        rr_references_[notId.second]->handleStatus(notId.first, status, message);
      }
      // leia rr, kuhu saata läbi query - DONE
      // target rr sees on meetod, mis otsib teised sõltuvused - DONE
      // calli rr user status callback
      // query liigutatakse edasi ja kordub sama - DONE
    }

    void handleStatus(const std::string &id, Status status, std::string &message)
    {
      std::cout << "<<<<<<<<<<<<<<<<<<<handleStatusSTART>>>>>>>>>>>>>>>>>>>" << std::endl;
      std::cout << "in handleStatus " << id << std::endl;
      std::cout << "rr - " << name_ << std::endl;
      std::string originQueryId = rr_catalog_->getOriginQueryId(id);

      if (originQueryId.size())
      {
        std::cout << "found upward id- " << originQueryId << std::endl;
        sendStatus(originQueryId, status, message);
      }
      std::cout << "<<<<<<<<<<<<<<<<<<<handleStatusEND>>>>>>>>>>>>>>>>>>>" << std::endl;
    }

  private:
    RrCatalogPtr rr_catalog_;
    std::string name_;

    RrServers<ServerType> servers_;
    RrClients<ClientType> clients_;

    std::unordered_map<std::string, StatusFunction> status_callbacks_;

    std::unordered_map<std::string, RrBase *> rr_references_;

    std::thread::id workId;
    std::mutex mtx;

    template <class CallClientClass, class ServType, class QueryType>
    void privateCall(const std::string *rr, RrBase *target, const std::string &server, QueryType &query, RrQueryBase *parentQuery, StatusFunction statusFunc, bool overrideFunc)
    {
      mtx.lock();

      workId = std::this_thread::get_id();

      query.setOrigin(name_);

      // In case we have a client call, not a internal call
      if ((rr == NULL) && !(target != NULL))
      {
        query.setRr(*(rr));
        handleClientCall<CallClientClass>(*(rr), server, query);
      }
      else
      {
        query.setRr(target->name());
        target->handleInternalCall<ServType, QueryType>(server, query);
      }

      if (parentQuery != NULL)
      {
        parentQuery->includeDependency(query.rr(), query.id());
      }

      std::cout << " LOLOLOLOLOLOLOLOLOL " << name_ << " - " << query.id() << std::endl;
      rr_catalog_->print();

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