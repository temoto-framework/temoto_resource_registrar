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

#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <stdio.h>
#include <thread>
#include <unordered_map>

#include "rr_catalog.h"
#include "rr_client_base.h"
#include "rr_configuration.h"
#include "rr_exceptions.h"
#include "rr_id_utils.h"
#include "rr_query_base.h"
#include "rr_server_base.h"
#include "rr_status.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <console_bridge/console.h>

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
      std::string error = "element '" + key + "' not found";
      throw ElementNotFoundException(error.c_str());
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
    std::unordered_map<std::string, std::unique_ptr<ContentClass>> rr_contents_;
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
    RrBase(const Configuration &config) : RrBase(config.name())
    {
      updateConfiguration(config);
    };

    RrBase(std::string name) : name_(name),
                               rr_catalog_(std::make_shared<RrCatalog>())
    {
      CONSOLE_BRIDGE_logInform("constructed rr %s", name.c_str());
    };

    /**
 * @brief Destroy the Rr Base object. Here we do the cleanup for backup storage and clients. Since unloadClient(const std::string &id)
 * uses virtual metods, one needs to create a destructor doing this logic for their implementations of RR. An example can be seen in 
 * rr_test.cpp TEST_F(RrBaseTest, ClientUnloadTest).
 * 
 */
    virtual ~RrBase()
    {
      CONSOLE_BRIDGE_logInform(("Destroying rr '" + name_ + "'").c_str());
      if (configuration_.eraseOnDestruct())
      {
        try
        {
          eraseSerializedCatalog();
        }
        catch (...)
        {
          CONSOLE_BRIDGE_logError("serializedCatalog erasure failed");
        }
      }
    }

    void updateConfiguration(const Configuration &config)
    {
      configuration_ = config;
      name_ = config.name();
    }

    const std::string id();

    void updateCatalog(const RrCatalog &catalog)
    {
      *rr_catalog_ = std::move(catalog);

      rr_catalog_->print();
    }

    void saveCatalog()
    {
      CONSOLE_BRIDGE_logDebug("saving catalog to: %s", (configuration_.location()).c_str());
      std::ofstream ofs(configuration_.location());
      boost::archive::binary_oarchive oa(ofs);
      oa << *(rr_catalog_.get());
      ofs.close();

      rr_catalog_->print();
    }

    void loadCatalog()
    {
      CONSOLE_BRIDGE_logDebug(" saving catalog to: %s", configuration_.location().c_str());
      std::ifstream ifs(configuration_.location(), std::ios::binary);
      boost::archive::binary_iarchive ia(ifs);
      RrCatalog catalog;
      ia >> catalog;

      updateCatalog(catalog);
    }

    template <class CallClientClass>
    void call(const std::string &rr, const std::string &server, RrQueryBase &query)
    {
      call<CallClientClass>(&rr, NULL, server, query);
    }

    template <class ServType, class QueryType, class StatusCallType>
    void call(RrBase &target,
              const std::string &server,
              QueryType &query,
              StatusCallType statusFunc = NULL,
              bool overrideStatus = false)
    {
      privateCall<RrClientBase, ServType, QueryType, StatusCallType>(NULL, &(target), server, query, statusFunc, overrideStatus);
    }

    template <class ServType, class QueryType>
    void call(RrBase &target,
              const std::string &server,
              QueryType &query)
    {
      privateCall<RrClientBase, ServType, QueryType, void *>(NULL, &(target), server, query, NULL, false);
    }

    size_t serverCount() { return servers_.getIds().size(); }
    size_t clientCount() { return clients_.getIds().size(); }

    virtual bool unload(const std::string &rr, const std::string &id)
    {
      throw NotImplementedException("'unload' is not implemented in the base class");
    }

    bool unload(RrBase &target, const std::string &id)
    {
      return target.localUnload(id);
    }

    bool localUnload(const std::string &id)
    {
      CONSOLE_BRIDGE_logDebug("localUnload id: %s", id.c_str());

      std::string serverId = rr_catalog_->getIdServer(id);

      CONSOLE_BRIDGE_logDebug("serverId id: %s", serverId.c_str());

      auto dependencyMap = rr_catalog_->getDependencies(id);
      if (dependencyMap.size() > 0)
      {
        CONSOLE_BRIDGE_logDebug("dependencyMap.size() > 0");
        for (auto const &dependency : dependencyMap)
        {
          unloadResource(id, dependency);
        }
      }
      bool res = unloadByServerAndQuery(serverId, id);
      autoSaveCatalog();
      return res;
    }

    void registerServer(std::unique_ptr<RrServerBase> serverPtr)
    {
      CONSOLE_BRIDGE_logInform("registering server");
      serverPtr->registerTransactionCb(std::bind(&RrBase::processTransactionCallback, this, std::placeholders::_1));
      serverPtr->initializeServer(name(), rr_catalog_);

      CONSOLE_BRIDGE_logInform("registration complete %s", (serverPtr->id()).c_str());
      servers_.add(std::move(serverPtr));
    }

    template <class ServType, class QueryType>
    void handleInternalCall(const std::string &server, QueryType &query)
    {
      CONSOLE_BRIDGE_logDebug("\t executing internal call to server: %s", server.c_str());
      auto &serverRef = servers_.getElement(server);

      auto dynamicRef = dynamic_cast<const ServType &>(serverRef);

      dynamicRef.processQuery(query);
    }

    void printCatalog() { rr_catalog_->print(); }

    std::string name() { return name_; }

    void setRrReferences(const std::unordered_map<std::string, RrBase *> &references) { rr_references_ = references; }

    std::string resolveQueryServerId(const std::string &id) { return rr_catalog_->getIdServer(id); }

    bool unloadByServerAndQuery(const std::string &server, const std::string &id) { return servers_.unload(server, id); }

    void sendStatus(Status statusData)
    {
      CONSOLE_BRIDGE_logDebug("core sendStatus %s", statusData.id_);

      std::unordered_map<std::string, std::string> notifyIds = rr_catalog_->getAllQueryIds(statusData.id_);
      for (auto const &notId : notifyIds)
      {
        std::string clientName = IDUtils::generateStatus(notId.second);
        CONSOLE_BRIDGE_logDebug("\t callStatusClient for client name %s", clientName.c_str());

        bool statusResult = callStatusClient(clientName, statusData);

        CONSOLE_BRIDGE_logDebug("\t call result: %i", statusResult);
      }
    }

    virtual void handleStatus(Status statusData)
    {
      CONSOLE_BRIDGE_logDebug("entered handleStatus %s", statusData.id_.c_str());
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
        CONSOLE_BRIDGE_logDebug("\t\tcalling callback of client %s", clientName.c_str());
        clients_.runCallback(clientName, statusData);
      }

      if (originalId.size())
      {
        statusData.id_ = originalId;

        CONSOLE_BRIDGE_logDebug("handleStatus");
        auto container = rr_catalog_->findOriginalContainer(statusData.id_);
        if (!container.empty_)
        {
          CONSOLE_BRIDGE_logDebug("\t\t\t!container.empty_");
          statusData.serialisedRequest_ = container.rawRequest_;
          statusData.serialisedRsponse_ = container.rawQuery_;
        }
        else
        {
          CONSOLE_BRIDGE_logDebug("\t\t\tcontainer.empty_");
        }

        CONSOLE_BRIDGE_logDebug("\t\tsendStatus to target %s", statusData.id_.c_str());

        std::async(&RrBase::sendStatus, this, statusData);
      }

      CONSOLE_BRIDGE_logDebug("-----exited handleStatus %s", statusData.id_.c_str());
    }

    // get target rr based on target server name and from there get all 
    // queries that used the clientName client
    std::map<std::string, std::string> getClientQueries(const std::string &serverName, const std::string &clientName)
    {
      CONSOLE_BRIDGE_logDebug("getClientQueries for server '%s' client '%s'", serverName.c_str(), clientName.c_str());
      // get server queries. can find the target RR from these queries
      std::string targetRr = rr_catalog_->getServerRr(serverName);
      CONSOLE_BRIDGE_logDebug("identified for targer rr to be: %s", targetRr.c_str());
      return callDataFetchClient(targetRr, IDUtils::generateServerName(targetRr, clientName));
    };

    std::map<UUID, std::string> handleDataFetch(const std::string &client)
    {
      CONSOLE_BRIDGE_logDebug("fetching query data for client: %s", client.c_str());

      std::map<UUID, std::string> ret;

      CONSOLE_BRIDGE_logDebug("getClientIds");

      std::set<std::string> ids = rr_catalog_->getServerIds(client);

      CONSOLE_BRIDGE_logDebug("findAndCollectIdQueries");
      findAndCollectIdQueries(ids, ret);

      CONSOLE_BRIDGE_logDebug("got %i queries ", ret.size());
      return ret;
    }

    virtual std::map<std::string, std::string> callDataFetchClient(const std::string &targetRr, const std::string &client)
    {
      CONSOLE_BRIDGE_logDebug("target rr for data fetch: %s", targetRr.c_str());
      auto res = rr_references_[targetRr]->handleDataFetch(client);
      CONSOLE_BRIDGE_logDebug("fetched %i queries", res.size());
      return res;
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
    std::string createClient(const std::string &rr, const std::string &server, QueryClass &query, const StatusCallType &statusCallback, bool overwriteCb)
    {
      std::string clientName = IDUtils::generateServerName(rr, server);

      bool oldClient = true;
      if (!clients_.exists(clientName))
      {
        CONSOLE_BRIDGE_logDebug("creating client! %s", clientName.c_str());
        std::unique_ptr<CallClientClass> client = std::make_unique<CallClientClass>(rr, server);
        CONSOLE_BRIDGE_logDebug("client created! %s", clientName.c_str());
        client->setCatalog(rr_catalog_);
        CONSOLE_BRIDGE_logDebug("Catalog set.");
        client->registerUserStatusCb(statusCallback);
        CONSOLE_BRIDGE_logDebug("CB registered.");
        clients_.add(std::move(client));
        CONSOLE_BRIDGE_logDebug("Client registered.");
        oldClient = false;
      }

      auto &client = clients_.getElement(clientName);
      CONSOLE_BRIDGE_logDebug("Getting client from storage.");
      auto dynamicRef = dynamic_cast<const CallClientClass &>(client);
      CONSOLE_BRIDGE_logDebug("dynamicRef done.");
      if (overwriteCb && (statusCallback != NULL) && oldClient)
      {
        dynamicRef.registerUserStatusCb(statusCallback);
      }

      return clientName;
    }

    virtual void unloadClient(const std::string &client)
    {
      CONSOLE_BRIDGE_logDebug("unloadClient %s", client.c_str());
      try
      {
        std::string targetRr = clients_.getElement(client).rr();
        clients_.remove(client);

        for (const std::string &id : rr_catalog_->getClientIds(client))
        {
          CONSOLE_BRIDGE_logDebug("\tunloadClient msg id %s", id.c_str());
          unload(targetRr, id);
        }
      }
      catch (const ElementNotFoundException &e)
      {
        throw ElementNotFoundException("Client not found");
      }
    }

    void unloadServer(const std::string &server)
    {
      throw NotImplementedException("'unloadServer' not implemented for RrBase");
    }

  protected:
    RrServers servers_;
    RrClients clients_;
    RrCatalogPtr rr_catalog_;

    Configuration configuration_;

    void updateQuery(const std::string &server,
                     const std::string &request,
                     const std::string &response)
    {
      rr_catalog_->updateResponse(server, request, response);
    }

    void autoSaveCatalog()
    {
      if (configuration_.saveOnModify())
      {
        std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
        saveCatalog();
      }
    }

    void eraseSerializedCatalog()
    {
      remove(configuration_.location().c_str());
    }

    virtual bool callStatusClient(const std::string &clientName, Status statusData)
    {
      throw NotImplementedException("'callStatusClient' not implemented for RrBase");
    }

    template <class CallClientClass, class QueryClass, class StatusCallType>
    void handleClientCall(const std::string &rr, const std::string clientName, QueryClass &query, const StatusCallType &statusCallback, bool overwriteCb)
    {
      std::string clientId = createClient<CallClientClass, QueryClass, StatusCallType>(rr, clientName, query, statusCallback, overwriteCb);

      auto client = dynamic_cast<const CallClientClass &>(clients_.getElement(clientId));
      client.invoke(query);

      std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
      rr_catalog_->storeClientCallRecord(clientId, query.id());
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
     * @param statusFunc 
     * @param overrideFunc 
     */
    template <class CallClientClass, class ServType, class QueryType, class StatusCallType>
    void privateCall(const std::string *rr, RrBase *target, const std::string &server, QueryType &query, const StatusCallType &statusFunc, bool overrideFunc)
    {

      std::thread::id workId = std::this_thread::get_id();

      CONSOLE_BRIDGE_logDebug("in private call");

      query.setOrigin(name_);

      CONSOLE_BRIDGE_logDebug("setting origin");

      std::string targetRrName, serverName;

      if ((rr != NULL) && (target == NULL))
        targetRrName = *(rr);
      else
        targetRrName = target->name();

      serverName = IDUtils::generateServerName(targetRrName, server);

      // In case we have a client call, not a internal call
      if ((rr != NULL) && (target == NULL))
      {
        CONSOLE_BRIDGE_logDebug("executing client call, also setting rr to %s", (*(rr)).c_str());
        query.setRr(targetRrName);
        handleClientCall<CallClientClass, QueryType, StatusCallType>(*(rr), server, query, statusFunc, overrideFunc);

        CONSOLE_BRIDGE_logDebug("query id: %s", query.id().c_str());
      }
      else
      {
        CONSOLE_BRIDGE_logDebug("executing mem call, also setting rr");
        query.setRr(targetRrName);
        target->handleInternalCall<ServType, QueryType>(serverName, query);

        CONSOLE_BRIDGE_logDebug("\t storeClientCallRecord to server: %s", server.c_str());

        rr_catalog_->storeClientCallRecord(serverName, query.id());
      }

      rr_catalog_->storeServerRr(serverName, targetRrName);

      if (!query.metadata().errorStack().empty())
      {
        CONSOLE_BRIDGE_logDebug("query had an error. unloading if dependencies exist");
        if (running_query_map_.count(workId))
        {
          CONSOLE_BRIDGE_logDebug("Dependencies might exist. Attempting unload");
          localUnload(running_query_map_[workId].id());
        }

        CONSOLE_BRIDGE_logDebug("throwing error upstream");
        throw FWD_TEMOTO_ERRSTACK(query.metadata().errorStack());
      }

      if (running_query_map_.count(workId))
      {
        CONSOLE_BRIDGE_logDebug("------------------------------------- has a dependency requirement");
        RrQueryBase bq = running_query_map_[workId];
        std::cout << "!!!Query " << query.id() << " is dependency of " << bq.id() << ". Stroring it" << std::endl;

        rr_catalog_->storeDependency(bq.id(), query.rr(), query.id());
      }

      autoSaveCatalog();
    }

    virtual void unloadResource(const std::string &id, const std::pair<const std::string, std::string> &dependency)
    {
      CONSOLE_BRIDGE_logDebug("private unloadResource() %s", id.c_str());
      std::string dependencyServer = rr_references_[dependency.second]->resolveQueryServerId(dependency.first);

      CONSOLE_BRIDGE_logDebug("dependencyServer %s", dependencyServer.c_str());

      bool unloadStatus = rr_references_[dependency.second]->unloadByServerAndQuery(dependencyServer, dependency.first);

      if (unloadStatus)
      {
        rr_catalog_->unloadDependency(id, dependency.first);
      }
    }

  private:
    std::string name_;
    std::unordered_map<std::string, RrBase *> rr_references_;
    mutable std::recursive_mutex modify_mutex_;

    // is a map of thread id - query objects. Used for automatic dependency detection
    std::unordered_map<std::thread::id, RrQueryBase> running_query_map_;

    void processTransactionCallback(const TransactionInfo &info)
    {
      std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
      std::thread::id workId = std::this_thread::get_id();
      // query started. Needs to be added to map
      if (info.type_ == 100)
      {
        running_query_map_[workId] = info.base_query_;
      }
      // query ended, needs removing from map
      else if (info.type_ == 200)
      {
        running_query_map_.erase(workId);
      }
    }

    void findAndCollectIdQueries(const std::set<std::string> &ids, std::map<UUID, std::string> &resultMap)
    {
      CONSOLE_BRIDGE_logDebug("findAndCollectIdQueries for %i ids", ids.size());

      for (const auto &id : ids)
      {
        QueryContainer<std::string> container = rr_catalog_->findOriginalContainer(id);
        if (!container.empty_)
        {
          resultMap[container.q_.id()] = container.rawRequest_;
        }
      }
    }
  };

} // namespace temoto_resource_registrar

#endif