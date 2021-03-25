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
#include "rr_configuration.h"
#include "rr_query_base.h"
#include "rr_server_base.h"
#include "rr_status.h"

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <fstream>

#include <mutex>

namespace temoto_resource_registrar
{
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

    const bool hasCallback(const std::string &key)
    {
      auto it = rr_contents_.find(key);
      if (it != rr_contents_.end())
      {
        return (it->second.get())->hasRegisteredCb();
      }
      return false;
    }

    const void runCallback(const std::string &key, const Status &statusInfo)
    {
      auto it = rr_contents_.find(key);
      if (it != rr_contents_.end())
      {
        (it->second.get())->internalStatusCallback(statusInfo);
      }
    }

  protected:
    std::unordered_map<std::string, std::unique_ptr<ContentClass>>
        rr_contents_;
  };

  class RrServers : public MapContainer<RrServerBase>
  {
  };

  class RrClients : public MapContainer<RrClientBase>
  {
  };

  /**
 * @brief 
 * 
 */
  class RrBase
  {
  public:
    RrBase(Configuration config) : RrBase(config.name())
    {
      configuration_ = config;
    };

    RrBase(std::string name) : name_(name),
                               rr_catalog_(std::make_shared<RrCatalog>()){};

    const std::string id();

    void updateCatalog(const RrCatalog &catalog)
    {
      rr_catalog_ = std::make_shared<RrCatalog>(catalog);
    }

    void saveCatalog()
    {
      std::cout << "saving catalog to: " << configuration_.location() << std::endl;
      std::ofstream ofs(configuration_.location());
      boost::archive::binary_oarchive oa(ofs);
      oa << *(rr_catalog_.get());
      ofs.close();
    }

    void loadCatalog()
    {
      std::cout << "loading catalog from: " << configuration_.location() << std::endl;
      std::ifstream ifs(configuration_.location(), std::ios::binary);
      boost::archive::binary_iarchive ia(ifs);
      RrCatalog catalog;
      ia >> catalog;

      updateCatalog(catalog);
    }

/*
    template <class QueryType>
    QueryType deSerializeBaseQuery(const QueryContainer<RawData> &container)
    {
      std::cout << " --deSerializeBaseQuery-- " << std::endl;

      std::stringstream ss(container.rawQuery_);
      boost::archive::binary_iarchive ia(ss);

      QueryType obj;
      ia >> obj;

      return obj;
    }

    template <class QueryType>
    std::vector<QueryType> getServerQueries(const std::string &server)
    {
      std::vector<QueryType> out;
      
      for (auto const &queryContainer : rr_catalog_->getUniqueServerQueries(server))
      {
        std::cout << " --1111111111111-- " << queryContainer.q_.id() << std::endl;
        out.push_back(deSerializeBaseQuery<QueryType>(queryContainer));
      }
      
      return out;
    }
    */

    template <class CallClientClass>
    void call(const std::string &rr, const std::string &server, RrQueryBase &query)
    {
      call<CallClientClass>(&rr, NULL, server, query);
    }

    template <class ServType, class QueryType, class StatusCallType>
    void call(RrBase &target,
              const std::string &server,
              QueryType &query,
              RrQueryBase *parentQuery = NULL,
              StatusCallType statusFunc = NULL,
              bool overrideStatus = false)
    {
      privateCall<RrClientBase, ServType, QueryType, StatusCallType>(NULL, &(target), server, query, parentQuery, statusFunc, overrideStatus);
    }

    template <class ServType, class QueryType>
    void call(RrBase &target,
              const std::string &server,
              QueryType &query,
              RrQueryBase *parentQuery = NULL)
    {
      privateCall<RrClientBase, ServType, QueryType, void *>(NULL, &(target), server, query, parentQuery, NULL, false);
    }

    size_t serverCount() { return servers_.getIds().size(); }

    bool unload(const std::string &rr, const std::string &id);

    bool unload(RrBase &target, const std::string &id)
    {
      return target.localUnload(id);
    }

    bool localUnload(const std::string &id)
    {
      std::cout << "localUnload id: " << id << std::endl;
      std::string serverId = rr_catalog_->getIdServer(id);

      std::cout << "serverId id: " << serverId << std::endl;

      auto dependencyMap = rr_catalog_->getDependencies(id);
      if (dependencyMap.size() > 0)
      {
        std::cout << "dependencyMap.size() > 0" << std::endl;
        for (auto const &dependency : dependencyMap)
        {
          unloadResource(id, dependency);
        }
      }
      return unloadByServerAndQuery(serverId, id);
    }

    void registerServer(std::unique_ptr<RrServerBase> serverPtr)
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

    void printCatalog() { rr_catalog_->print(); }

    std::string name() { return name_; }

    void setRrReferences(const std::unordered_map<std::string, RrBase *> &references) { rr_references_ = references; }

    std::string resolveQueryServerId(const std::string &id) { return rr_catalog_->getIdServer(id); }

    bool unloadByServerAndQuery(const std::string &server, const std::string &id) { return servers_.unload(server, id); }

    virtual void sendStatus(Status statusData)
    {
      std::cout << "core sendStatus " << statusData.id_ << std::endl;
      std::unordered_map<std::string, std::string> notifyIds = rr_catalog_->getAllQueryIds(statusData.id_);
      for (auto const &notId : notifyIds)
      {
        std::string clientName = notId.second + "_status";
        std::cout << "\t callStatusClient for client name" << clientName << std::endl;
        bool statusResult = callStatusClient(clientName, statusData);
        std::cout << "\t call result: " << statusResult << std::endl;
      }
    }

    virtual void handleStatus(Status statusData)
    {
      std::cout << "-----entered handleStatus " << statusData.id_ << std::endl;
      std::string originalId = rr_catalog_->getOriginQueryId(statusData.id_);
      if (originalId.size())
      {
        std::unordered_map<std::string, std::string> dependencyMap = rr_catalog_->getDependencies(originalId);
        std::string firstId = dependencyMap.begin()->first;
        // stop attempt if dependency
        if (statusData.id_ != firstId)
        {
          return;
        }
      }

      std::string clientName = rr_catalog_->getIdClient(statusData.id_);

      if (clients_.exists(clientName))
      {
        std::cout << "\t\tcalling callback of client " << clientName << std::endl;
        clients_.runCallback(clientName, statusData);
      }

      if (originalId.size())
      {
        statusData.id_ = originalId;

        auto container = rr_catalog_->findOriginalContainer(statusData.id_);
        if (!container.empty_)
        {
          std::cout << "\t\t\t!container.empty_" << std::endl;
          statusData.serialisedRequest_ = container.rawRequest_;
          statusData.serialisedRsponse_ = container.rawQuery_;
        }
        else
        {
          std::cout << "\t\t\tcontainer.empty_" << std::endl;
        }

        std::cout << "\t\tsendStatus to target " << statusData.id_ << std::endl;
        sendStatus(statusData);
      }
      std::cout << "-----exited handleStatus " << statusData.id_ << std::endl;
    }

    std::vector<std::string> callbacks()
    {
      std::vector<std::string> cbVector;
      int counter = 0;
      for (const std::string &id : clients_.getIds())
      {
        if (clients_.hasCallback(id))
          cbVector.push_back(id);
      }

      return cbVector;
    }

    /**
 * @brief Create a Client object. Used to initializing clients for a RR. If a client is created it will be used for calls to target servers
 * and a client entity is not created when a call is executed.
 * 
 * @tparam CallClientClass 
 * @tparam QueryClass 
 * @tparam StatusCallType 
 * @param rr 
 * @param server 
 * @param query 
 * @param statusCallback 
 * @param overwriteCb 
 */
    template <class CallClientClass, class QueryClass, class StatusCallType>
    void createClient(const std::string &rr, const std::string &server, QueryClass &query, const StatusCallType &statusCallback, bool overwriteCb)
    {
      std::string clientName = rr + "_" + server;
      bool oldClient = true;
      if (!clients_.exists(clientName))
      {
        std::cout << "creating client! " << clientName << std::endl;

        std::unique_ptr<CallClientClass> client = std::make_unique<CallClientClass>(clientName);
        client->setCatalog(rr_catalog_);

        client->registerUserStatusCb(statusCallback);

        clients_.add(std::move(client));

        oldClient = false;
      }

      auto &client = clients_.getElement(clientName);
      auto dynamicRef = dynamic_cast<const CallClientClass &>(client);

      if (overwriteCb && (statusCallback != NULL) && oldClient)
      {
        dynamicRef.registerUserStatusCb(statusCallback);
      }
    }

  protected:
    RrServers servers_;
    RrClients clients_;
    RrCatalogPtr rr_catalog_;

    Configuration configuration_;

    virtual bool callStatusClient(const std::string &clientName, Status statusData){};

    template <class CallClientClass, class QueryClass, class StatusCallType>
    void handleClientCall(const std::string &rr, const std::string &server, QueryClass &query, const StatusCallType &statusCallback, bool overwriteCb)
    {
      std::string clientName = rr + "_" + server;

      createClient<CallClientClass, QueryClass, StatusCallType>(rr, server, query, statusCallback, overwriteCb);

      auto client = dynamic_cast<const CallClientClass &>(clients_.getElement(clientName));
      client.invoke(query);

      rr_catalog_->storeClientCallRecord(clientName, query.id());
    }

    /**
     * @brief TODO: requires a proper comment.
     * Note: If your application has multiple client calls to different services, and each call has its unique status
     * callback, then make sure that the appropriate callback is provided. Otherwise they get mixed and thats probably
     * not the intended behavior.
     * Since the status callback function is common across all client calls, make sure that 
     * 
     * @tparam CallClientClass 
     * @tparam ServType 
     * @tparam QueryType 
     * @param rr 
     * @param target 
     * @param server 
     * @param query 
     * @param parentQuery 
     * @param statusFunc 
     * @param overrideFunc 
     */
    template <class CallClientClass, class ServType, class QueryType, class StatusCallType>
    void privateCall(const std::string *rr, RrBase *target, const std::string &server, QueryType &query, RrQueryBase *parentQuery, const StatusCallType &statusFunc, bool overrideFunc)
    {
      mtx.lock();

      workId = std::this_thread::get_id();

      std::cout << "in private call" << std::endl;

      query.setOrigin(name_);

      std::cout << "setting origin" << std::endl;

      // In case we have a client call, not a internal call
      if ((rr != NULL) && (target == NULL))
      {
        std::cout << "executing client call, also setting rr to " << *(rr) << std::endl;
        query.setRr(*(rr));
        handleClientCall<CallClientClass, QueryType, StatusCallType>(*(rr), server, query, statusFunc, overrideFunc);
        std::cout << "query id: " << query.id() << std::endl;
      }
      else
      {
        std::cout << "executing mem call, also setting rr" << std::endl;
        query.setRr(target->name());
        target->handleInternalCall<ServType, QueryType>(server, query);
      }

      if (parentQuery != NULL)
      {
        std::cout << "if has parent query: " << query.rr() << "-" << query.id() << std::endl;
        parentQuery->includeDependency(query.rr(), query.id());
      }

      mtx.unlock();
    }

    virtual void unloadResource(const std::string &id, const std::pair<const std::string, std::string> &dependency)
    {
      std::cout << "private unloadResource() " << id << std::endl;
      std::string dependencyServer = rr_references_[dependency.second]->resolveQueryServerId(dependency.first);
      std::cout << "dependencyServer " << dependencyServer << std::endl;
      bool unloadStatus = rr_references_[dependency.second]->unloadByServerAndQuery(dependencyServer, dependency.first);

      if (unloadStatus)
      {
        rr_catalog_->unloadDependency(id, dependency.first);
      }
    }

  private:
    std::string name_;
    std::unordered_map<std::string, RrBase *> rr_references_;
    std::thread::id workId;
    std::mutex mtx;
  };

} // namespace temoto_resource_registrar

#endif