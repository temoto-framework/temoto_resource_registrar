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

#include "temoto_resource_registrar/rr_catalog.h"

namespace temoto_resource_registrar
{

  void RrCatalog::storeQuery(const std::string &server, RrQueryBase q, RawData reqData, RawData qData)
  {
    

    std::cout << "in store query.." << std::endl;

    std::lock_guard<std::mutex> lock(modify_mutex_);
    id_query_map_[reqData] = QueryContainer<RawData>(q, reqData, qData, server);
    server_id_map_[server].insert(q.id());

    std::cout << "storage done..." << std::endl;
  }

  void RrCatalog::updateResponse(const std::string &server, RawData request, RawData response)
  {
    std::lock_guard<std::mutex> lock(modify_mutex_);

    RawData key = "";
    for (auto const &query : id_query_map_)
    {
      QueryContainer<RawData> wrapper = query.second;
      if (wrapper.rawRequest_ == request && wrapper.responsibleServer_ == server)
      {
        key = query.first;
        break;
      }
    }

    if (key.size())
    {
      id_query_map_[key].rawQuery_ = response;
    }
  }

  UUID RrCatalog::queryExists(const std::string &server, RawData reqData)
  {
    std::lock_guard<std::mutex> lock(modify_mutex_);
    for (auto const &query : id_query_map_)
    {
      QueryContainer<RawData> wrapper = query.second;

      std::cout << (wrapper.rawRequest_ == reqData) << " & " << (wrapper.responsibleServer_ == server) << std::endl;

      if (wrapper.rawRequest_ == reqData && wrapper.responsibleServer_ == server)
      {
        return wrapper.q_.id();
      }
    }
    return "";
  }

  RawData RrCatalog::processExisting(const std::string &server, const std::string &id, RrQueryBase q)
  {
    std::string request = "";

    std::cout << "processExisting" << std::endl;

    
    request = findOriginalContainer(id).rawRequest_;

    std::lock_guard<std::mutex> lock(modify_mutex_);
    if (request.size())
    {
      id_query_map_[request].storeNewId(q.id(), q.origin());
      server_id_map_[server].insert(q.id());
      return id_query_map_[request].rawQuery_;
    }

    return "";
  }

  UUID RrCatalog::getInitialId(const std::string &id)
  {
    //QueryContainer cotnainer = findOriginalContainer(id);
    //std::cout << cotnainer.getIdCount() << std::endl;
    //return cotnainer.q_.id();

    std::cout << "getInitialId" << std::endl;
    return getOriginQueryId(id);
  }

  RawData RrCatalog::unload(const std::string &server, const std::string &id, bool &unloadable)
  {

    std::lock_guard<std::mutex> lock(modify_mutex_);
    auto vec = server_id_map_[server];
    int removedElCnt = vec.erase(id);
    server_id_map_[server] = vec;

    std::string queryResp = "";

    modify_mutex_.unlock();
    if (removedElCnt)
    {
      std::cout << "unload" << std::endl;
      QueryContainer<RawData> qc = findOriginalContainer(id);

      if (qc.responsibleServer_.size())
      {
        modify_mutex_.lock();
        queryResp = qc.rawQuery_;
        qc.removeId(id);

        if (!qc.getIdCount())
        {
          unloadable = true;
          id_query_map_.erase(qc.rawRequest_);
        }
        else
        {
          id_query_map_[qc.rawRequest_] = qc;
        }
      }
    }

    if (!server_id_map_[server].size())
      server_id_map_.erase(server);

    return queryResp;
  }

  QueryContainer<RawData> RrCatalog::findOriginalContainer(const std::string &id)
  {
    std::cout << "findOriginalContainer: " << id << std::endl;
    std::lock_guard<std::mutex> lock(modify_mutex_);
    for (auto const &queryEntry : id_query_map_)
    {
      if (queryEntry.second.rr_ids_.count(id))
      {
        std::cout << "Found!: " << queryEntry.second.q_.id() << std::endl;
        return queryEntry.second;
      }
    }

    return QueryContainer<RawData>();
  }

  ServerName RrCatalog::getIdServer(const std::string &id)
  {
    std::cout << "getIdServer: " << id << std::endl;
    return findOriginalContainer(id).responsibleServer_;
  }

  std::unordered_map<UUID, std::string> RrCatalog::getAllQueryIds(const std::string &id)
  {
    std::lock_guard<std::mutex> lock(modify_mutex_);
    for (auto const &queryEntry : id_query_map_)
    {
      if (queryEntry.second.rr_ids_.count(id))
      {
        return queryEntry.second.rr_ids_;
      }
    }
    std::unordered_map<UUID, std::string> emptyMap;
    return emptyMap;
  }

  void RrCatalog::storeDependency(const std::string &queryId, const std::string &dependencySource, const std::string &dependencyId)
  {
    std::lock_guard<std::mutex> lock(modify_mutex_);
    id_dependency_map_[queryId].registerDependency(dependencySource, dependencyId);
  }

  std::unordered_map<UUID, std::string> RrCatalog::getDependencies(const std::string &queryId)
  {
    std::lock_guard<std::mutex> lock(modify_mutex_);
    if (id_dependency_map_.count(queryId))
      return id_dependency_map_[queryId].dependencies();

    std::unordered_map<std::string, std::string> empty;
    return empty;
  }

  void RrCatalog::unloadDependency(const std::string &queryId, const std::string &dependencyId)
  {
    std::lock_guard<std::mutex> lock(modify_mutex_);
    id_dependency_map_[queryId].removeDependency(dependencyId);

    if (!id_dependency_map_[queryId].count())
    {
      id_dependency_map_.erase(queryId);
    }
  }

  UUID RrCatalog::getOriginQueryId(const std::string &queryId)
  {
    std::lock_guard<std::mutex> lock(modify_mutex_);
    for (auto const &dependencyEntry : id_dependency_map_)
    {
      if (dependencyEntry.second.dependencies().count(queryId))
      {
        return dependencyEntry.first;
      }
    }
    return "";
  }

  void RrCatalog::storeClientCallRecord(const std::string &client, const std::string &id)
  {
    std::lock_guard<std::mutex> lock(modify_mutex_);
    client_id_map_[client].insert(id);
  }

  ClientName RrCatalog::getIdClient(const std::string &id)
  {
    std::lock_guard<std::mutex> lock(modify_mutex_);
    for (auto const &clientQueries : client_id_map_)
    {
      if (clientQueries.second.count(id))
      {
        return clientQueries.first;
      }
    }
    return "";
  }

  std::vector<QueryContainer<RawData>> RrCatalog::getUniqueServerQueries(const std::string &server)
  {
    std::vector<QueryContainer<RawData>> output;
    std::set<UUID> addedMessages;
    std::lock_guard<std::mutex> lock(modify_mutex_);
    for (auto const &queryId : server_id_map_[server])
    {
      std::cout << "getUniqueServerQueries" << std::endl;
      QueryContainer<RawData> container = findOriginalContainer(queryId);
      if (addedMessages.count(container.q_.id()) == 0)
      {
        output.push_back(container);
        addedMessages.insert(container.q_.id());
      }
    }

    return output;
  }

  void RrCatalog::print()
  {
    std::cout << "server_id_map_: " << std::endl;
    for (auto const &i : server_id_map_)
    {
      std::cout << "{" << std::endl;
      std::cout << i.first << ": ";
      for (auto const &j : i.second)
      {
        std::cout << j << ", ";
      }
      std::cout << std::endl;
      std::cout << "}" << std::endl;
    }

    std::cout << "id_query_map_: " << std::endl;
    for (auto const &i : id_query_map_)
    {
      std::cout << "{" << std::endl;
      std::cout << i.first << ": ";
      for (auto const &j : i.second.rr_ids_)
      {
        std::cout << j.first << ": " << j.second << "; ";
      }
      std::cout << std::endl;
      std::cout << "}" << std::endl;
    }

    std::cout << "id_dependency_map: " << std::endl;
    for (auto const &i : id_dependency_map_)
    {
      std::cout << "{" << std::endl;
      std::cout << i.first << ": ";

      i.second.print();

      std::cout << std::endl;
      std::cout << "}" << std::endl;
    }
  }

} // namespace temoto_resource_registrar