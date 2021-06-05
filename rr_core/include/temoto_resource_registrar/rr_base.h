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
#include <utility>

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

    ContentClass *getElementPtr(const std::string &key)
    {
      auto it = rr_contents_.find(key);
      if (it != rr_contents_.end())
      {
        return (it->second.get());
      }
      std::string error = "element '" + key + "' not found";
      throw ElementNotFoundException(error.c_str());
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

    const bool hasCallback(const std::string &key, const std::string &queryId)
    {
      auto it = rr_contents_.find(key);
      if (it != rr_contents_.end())
      {
        return (it->second.get())->hasRegisteredCb(queryId);
      }
      return false;
    }

    const void runCallback(const std::string &key, const std::string &queryId, const Status &statusInfo)
    {
      auto it = rr_contents_.find(key);
      if (it != rr_contents_.end())
      {
        (it->second.get())->internalStatusCallback(queryId, statusInfo);
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
      //TEMOTO_INFO_("constructed rr %s", name.c_str());
    };

    /**
 * @brief Destroy the Rr Base object. Here we do the cleanup for backup storage and clients. Since unloadClient(const std::string &id)
 * uses virtual metods, one needs to create a destructor doing this logic for their implementations of RR. An example can be seen in 
 * rr_test.cpp TEST_F(RrBaseTest, ClientUnloadTest).
 * 
 */
    virtual ~RrBase()
    {
      ////TEMOTO_INFO_(("Destroying rr '" + name_ + "'").c_str());
      if (configuration_.eraseOnDestruct())
      {
        try
        {
          eraseSerializedCatalog();
        }
        catch (...)
        {
          //CONSOLE_BRIDGE_logError("serializedCatalog erasure failed");
        }
      }
    }

    void updateConfiguration(const Configuration &config)
    {
      configuration_ = config;
      name_ = config.name();
      rr_catalog_->updateConfiguration(configuration_);
    }

    const std::string id();

    void updateCatalog(const RrCatalog &catalog)
    {
      *rr_catalog_ = std::move(catalog);

      rr_catalog_->updateConfiguration(configuration_);
    }

    void saveCatalog()
    {
      rr_catalog_->saveCatalog();
    }

    void loadCatalog()
    {
      //TEMOTO_DEBUG_(" saving catalog to: %s", configuration_.location().c_str());
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
              StatusCallType status_func = NULL)
    {
      privateCall<RrClientBase, ServType, QueryType, StatusCallType>(NULL, &(target), server, query, status_func);
    }

    template <class ServType, class QueryType>
    void call(RrBase &target,
              const std::string &server,
              QueryType &query)
    {
      privateCall<RrClientBase, ServType, QueryType, void *>(NULL, &(target), server, query, NULL);
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
      //TEMOTO_DEBUG_("localUnload id: %s", id.c_str());

      std::string serverId = rr_catalog_->getIdServer(id);

      //TEMOTO_DEBUG_("serverId id: %s", serverId.c_str());

      auto dependency_map = rr_catalog_->getDependencies(id);
      if (dependency_map.size() > 0)
      {
        //TEMOTO_DEBUG_("dependencyMap.size() > 0");
        for (auto const &dependency : dependency_map)
        {
          unloadResource(id, dependency);
        }
      }
      bool res = unloadByServerAndQuery(serverId, id);
      autoSaveCatalog();
      return res;
    }

    void registerServer(std::unique_ptr<RrServerBase> server_ptr)
    {
      //TEMOTO_INFO_("registering server");
      server_ptr->registerTransactionCb(std::bind(&RrBase::processTransactionCallback, this, std::placeholders::_1));
      server_ptr->initializeServer(name(), rr_catalog_);

      //TEMOTO_INFO_("registration complete %s", (server_ptr->id()).c_str());
      servers_.add(std::move(server_ptr));
    }

    template <class ServType, class QueryType>
    void handleInternalCall(const std::string &server, QueryType &query)
    {
      //TEMOTO_DEBUG_("\t executing internal call to server: %s", server.c_str());
      //auto &server_ref = servers_.getElement(server);

      //auto dynamic_ref = dynamic_cast<const ServType &>(server_ref);

      //dynamic_ref.processQuery(query);
      reinterpret_cast<ServType *>(servers_.getElementPtr(server))->processQuery(query);

      
    }

    void printCatalog() { rr_catalog_->print(); }

    std::string name() { return name_; }

    void setRrReferences(const std::unordered_map<std::string, RrBase *> &references) { rr_references_ = references; }

    std::string resolveQueryServerId(const std::string &id) { return rr_catalog_->getIdServer(id); }

    bool unloadByServerAndQuery(const std::string &server, const std::string &id) { return servers_.unload(server, id); }

    void sendStatus(const std::string &quiery_id, Status status_data)
    {
      ////TEMOTO_DEBUG_("core sendStatus %s", status_data.id_);

      std::unordered_map<std::string, std::string> notify_ids = rr_catalog_->getAllQueryIds(status_data.id_);
      for (auto const &not_id : notify_ids)
      {
        ////TEMOTO_DEBUG_("\t callStatusClient for rr %s", not_id.second.c_str());

        bool status_result = callStatusClient(not_id.second, quiery_id, status_data);

        ////TEMOTO_DEBUG_("\t call result: %i", status_result);
      }
    }

    virtual void handleStatus(const std::string &request_id, Status status_data)
    {
      TEMOTO_DEBUG_("entered handleStatus %s - %s - %s", name().c_str(), request_id.c_str(), status_data.id_.c_str());

      std::string original_id = rr_catalog_->getOriginQueryId(status_data.id_);
      std::string client_name = rr_catalog_->getIdClient(status_data.id_);
      TEMOTO_DEBUG_("query id %s", original_id.c_str());

      if (clients_.exists(client_name))
      {
        TEMOTO_DEBUG_("calling callback of client %s", client_name.c_str());
        clients_.runCallback(client_name, request_id, status_data);
      }

      if (original_id.size())
      {
        status_data.id_ = original_id;

        TEMOTO_DEBUG_("handleStatus");
        auto container = rr_catalog_->findOriginalContainer(status_data.id_);
        if (!container.empty_)
        {
          TEMOTO_DEBUG_("!container.empty_");
          status_data.serialised_request_ = container.raw_request_;
          status_data.serialised_response_ = container.raw_query_;
        }
        else
        {
          TEMOTO_DEBUG_("container.empty_");
        }

        TEMOTO_DEBUG_("sendStatus to target %s", status_data.id_.c_str());

        std::async(&RrBase::sendStatus, this, original_id, status_data);
      }

      TEMOTO_DEBUG_("-----exited handleStatus %s", status_data.id_.c_str());
    }

    std::map<std::string, std::pair<std::string, std::string>> getChildQueries(const std::string &id, const std::string &server_name)
    {
      std::map<std::string, std::pair<std::string, std::string>> res;

      QueryContainer<std::string> q_container = rr_catalog_->findOriginalContainer(id);

      std::string client_name;

      if (q_container.empty_)
      {
        //TEMOTO_DEBUG_("Could not find base container. Maybe is pure client Rr. They can not have multiple dependencies since call executes a single query");
        return res;
      }
      else
      {
      }
      // UUID - servingRR
      std::unordered_map<std::string, std::string> dependencies = rr_catalog_->getDependencies(q_container.q_.id());

      //TEMOTO_DEBUG_("Dependencies:");
      for (const auto &dep : dependencies)
      {
        //TEMOTO_DEBUG_("%s - %s", dep.first.c_str(), dep.second.c_str());

        //leia client mis seda resurssi haldas
        client_name = rr_catalog_->getIdClient(dep.first);

        if (IDUtils::generateServerName(dep.second, server_name) == client_name)
        {
          //TEMOTO_DEBUG_("client name %s matched. Fetching queries", client_name.c_str());
          res = getServerRrQueries(client_name, name());
        }
      }

      return res;
    }

    std::map<UUID, std::pair<std::string, std::string>> handleDataFetch(const std::string &origin_rr, const std::string &server_name)
    {
      //TEMOTO_DEBUG_("fetching query data for server name: %s", server_name.c_str());

      std::map<UUID, std::pair<std::string, std::string>> ret;

      //TEMOTO_DEBUG_("getClientIds");

      std::set<std::string> ids = rr_catalog_->getServerIds(server_name);

      //TEMOTO_DEBUG_("findAndCollectIdQueries");
      findAndCollectIdQueries(ids, origin_rr, ret);

      //TEMOTO_DEBUG_("got %i queries ", ret.size());
      return ret;
    }

    virtual std::map<std::string,
                     std::pair<std::string, std::string>>
    callDataFetchClient(const std::string &target_rr,
                        const std::string &origin_rr,
                        const std::string &server_name)
    {
      //TEMOTO_DEBUG_("target rr for data fetch: %s", target_rr.c_str());
      auto res = rr_references_[target_rr]->handleDataFetch(origin_rr, server_name);
      //TEMOTO_DEBUG_("fetched %i queries", res.size());
      return res;
    }

    std::vector<std::string> callbacks()
    {
      std::vector<std::string> cb_vector;
      int counter = 0;
      for (const std::string &id : clients_.getIds())
      {
        for (const std::string &q_id : clients_.getElement(id).registeredCallbackQueries())
        {
          cb_vector.push_back(q_id);
        }
      }

      return cb_vector;
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
 */
    template <class CallClientClass>
    std::string createClient(const std::string &rr,
                             const std::string &server)
    {
      std::string client_name = IDUtils::generateServerName(rr, server);

      if (!clients_.exists(client_name))
      {
        //TEMOTO_DEBUG_("creating client! %s", client_name.c_str());
        std::unique_ptr<CallClientClass> client = std::make_unique<CallClientClass>(rr, server);
        //TEMOTO_DEBUG_("client created! %s", client_name.c_str());
        client->setCatalog(rr_catalog_);
        //TEMOTO_DEBUG_("Catalog set.");
        clients_.add(std::move(client));
        //TEMOTO_DEBUG_("Client registered.");
      }

      return client_name;
    }

    template <class CallClientClass, class StatusCallType>
    void storeClientQueryStatusCb(const std::string client_id, const std::string &query_id, const StatusCallType &status_callback)
    {
      if (status_callback != NULL && query_id.size() > 0)
      {
        auto client = dynamic_cast<CallClientClass *>(clients_.getElementPtr(client_id));
        client->registerUserStatusCb(query_id, status_callback);
      }
    }

    virtual void unloadClient(const std::string &client)
    {
      //TEMOTO_DEBUG_("unloadClient %s", client.c_str());
      try
      {
        std::string target_rr = clients_.getElement(client).rr();
        clients_.remove(client);

        for (const std::string &id : rr_catalog_->getClientIds(client))
        {
          //TEMOTO_DEBUG_("\tunloadClient msg id %s", id.c_str());
          unload(target_rr, id);
        }
      }
      catch (const ElementNotFoundException &e)
      {
        throw ElementNotFoundException(("unloadClient " + client + " not found").c_str());
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

    void handleRrServerCb(const std::string &query_id, const Status &status)
    {
      std::string server_name = rr_catalog_->getIdServer(query_id);
      ////TEMOTO_DEBUG_("running server status cb Maybe it is server %s", server_name.c_str());
      try
      {
        servers_.getElement(server_name).triggerCallback(status);
      }
      catch (const ElementNotFoundException &e)
      {
        ////TEMOTO_DEBUG_("Server not found. Continuing");
      }
    }

    // get target rr based on target server name and from there get all
    // queries that used the clientName client
    std::map<std::string, std::pair<std::string, std::string>> getServerRrQueries(const std::string &server_name, const std::string &query_rr)
    {
      //TEMOTO_DEBUG_("getClientQueries for server '%s' from rr '%s'", server_name.c_str(), queryRr.c_str());
      // get server queries. can find the target RR from these queries
      std::string target_rr = rr_catalog_->getServerRr(server_name);
      //TEMOTO_DEBUG_("identified for targer rr to be: %s", targetRr.c_str());
      return callDataFetchClient(target_rr, query_rr, server_name);
    };

    void updateQuery(const std::string &server,
                     const std::string &request,
                     const std::string &response)
    {
      rr_catalog_->updateResponse(IDUtils::generateServerName(name(), server), request, response);
      autoSaveCatalog();
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

    virtual bool callStatusClient(const std::string &target_rr, const std::string &request_id, Status status_data)
    {
      ////TEMOTO_DEBUG_("target rr for status: %s", target_rr.c_str());

      auto container = rr_catalog_->findOriginalContainer(status_data.id_);

      if (!container.empty_)
      {
        status_data.serialised_request_ = container.raw_request_;
        status_data.serialised_response_ = container.raw_query_;
      }

      handleRrServerCb(request_id, status_data);

      rr_references_[target_rr]->handleStatus(request_id, status_data);

      return true;
    }

    template <class CallClientClass, class QueryClass, class StatusCallType>
    void handleClientCall(const std::string &rr, const std::string client_name,
                          QueryClass &query,
                          const StatusCallType &status_callback)
    {
      std::string client_id = createClient<CallClientClass>(rr, client_name);

      reinterpret_cast<CallClientClass *>(clients_.getElementPtr(client_id))->invoke(query);

      storeClientQueryStatusCb<CallClientClass, StatusCallType>(client_id, query.id(), status_callback);

      std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
      rr_catalog_->storeClientCallRecord(client_id, query.id());
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
     * @param status_callback
     */
    template <class CallClientClass, class ServType, class QueryType, class StatusCallType>
    void privateCall(const std::string *rr, RrBase *target, const std::string &server, QueryType &query, const StatusCallType &status_callback)
    { 
      START_SPAN

      std::thread::id work_id = std::this_thread::get_id();

      //TEMOTO_DEBUG_("in private call");

      query.setOrigin(name_);

      //TEMOTO_DEBUG_("setting origin");

      std::string target_rr_name, server_name;

      if ((rr != NULL) && (target == NULL))
        target_rr_name = *(rr);
      else
        target_rr_name = target->name();

      server_name = IDUtils::generateServerName(target_rr_name, server);

      #ifdef temoto_enable_tracing
      query.requestMetadata().setSpanContext(TEMOTO_LOG_ATTR.topParentSpanContext());
      #endif

      // In case we have a client call, not a internal call
      if ((rr != NULL) && (target == NULL))
      {
        //TEMOTO_DEBUG_("executing client call, also setting rr to %s", (*(rr)).c_str());
        query.setRr(target_rr_name);
        handleClientCall<CallClientClass, QueryType, StatusCallType>(*(rr), server, query, status_callback);

        //TEMOTO_DEBUG_("query id: %s", query.id().c_str());
      }
      else
      {
        //TEMOTO_DEBUG_("executing mem call, also setting rr");
        query.setRr(target_rr_name);
        target->handleInternalCall<ServType, QueryType>(server_name, query);

        //TEMOTO_DEBUG_("\t storeClientCallRecord to server: %s", server.c_str());

        rr_catalog_->storeClientCallRecord(server_name, query.id());
      }

      rr_catalog_->storeServerRr(server_name, target_rr_name);

      if (!query.responseMetadata().errorStack().empty())
      {
        //TEMOTO_DEBUG_("query had an error. unloading if dependencies exist");
        if (running_query_map_.count(work_id))
        {
          //TEMOTO_DEBUG_("Dependencies might exist. Attempting unload");
          localUnload(running_query_map_[work_id].id());
        }

        //TEMOTO_DEBUG_("throwing error upstream");
        throw FWD_TEMOTO_ERRSTACK(query.responseMetadata().errorStack());
      }

      if (running_query_map_.count(work_id))
      {
        //TEMOTO_DEBUG_("------------------------------------- has a dependency requirement");
        RrQueryBase bq = running_query_map_[work_id];
        std::cout << "!!!Query " << query.id() << " is dependency of " << bq.id() << ". Stroring it" << std::endl;

        rr_catalog_->storeDependency(bq.id(), query.rr(), query.id());
      }

      autoSaveCatalog();
    }

    virtual void unloadResource(const std::string &id, const std::pair<const std::string, std::string> &dependency)
    {
      //TEMOTO_DEBUG_("private unloadResource() %s", id.c_str());
      std::string dependency_server = rr_references_[dependency.second]->resolveQueryServerId(dependency.first);

      //TEMOTO_DEBUG_("dependencyServer %s", dependency_server.c_str());

      bool unload_status = rr_references_[dependency.second]->unloadByServerAndQuery(dependency_server, dependency.first);

      if (unload_status)
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
      std::thread::id work_id = std::this_thread::get_id();
      // query started. Needs to be added to map
      if (info.type_ == 100)
      {
        running_query_map_[work_id] = info.base_query_;
      }
      // query ended, needs removing from map
      else if (info.type_ == 200)
      {
        running_query_map_.erase(work_id);
      }
    }

    void findAndCollectIdQueries(const std::set<std::string> &ids, const std::string &origin_rr, std::map<UUID, std::pair<std::string, std::string>> &result_map)
    {
      //TEMOTO_DEBUG_("findAndCollectIdQueries for %i ids. Origin: %s", ids.size(), origin_rr.c_str());

      for (const auto &id : ids)
      {
        QueryContainer<std::string> container = rr_catalog_->findOriginalContainer(id);
        //TEMOTO_DEBUG_("origin of container: %s", container.q_.origin().c_str());
        if (!container.empty_ && container.q_.origin() == origin_rr)
        {
          std::pair<std::string, std::string> request_response_pair(container.raw_request_, container.raw_query_);
          result_map[container.q_.id()] = request_response_pair;
        }
      }
    }
  };

} // namespace temoto_resource_registrar

#endif