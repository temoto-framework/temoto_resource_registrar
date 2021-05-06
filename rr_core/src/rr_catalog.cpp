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

  void RrCatalog::storeQuery(const std::string &server,
                             RrQueryBase q,
                             RawData request_data,
                             RawData query_data)
  {

    std::cout << "in store query.." << std::endl;

    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    std::cout << "locking..." << std::endl;
    id_query_map_[request_data] = QueryContainer<RawData>(q, request_data, query_data, server);
    std::cout << "id_query_map_[request_data] set" << std::endl;
    server_id_map_[server].insert(q.id());
    std::cout << "server_id_map_[server] set" << std::endl;

    std::cout << "storage done... current state of catalog:" << std::endl;
    print();
  }

  void RrCatalog::updateResponse(const std::string &server, RawData request, RawData response)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);

    RawData key = "";
    for (auto const &query : id_query_map_)
    {
      QueryContainer<RawData> wrapper = query.second;
      if (wrapper.raw_request_ == request && wrapper.responsible_server_ == server)
      {
        key = query.first;
        break;
      }
    }

    if (key.size())
    {
      id_query_map_[key].raw_query_ = response;
    }
  }

  UUID RrCatalog::queryExists(const std::string &server, RawData request_data)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    for (auto const &query : id_query_map_)
    {
      QueryContainer<RawData> wrapper = query.second;

      if (wrapper.raw_request_ == request_data &&
          wrapper.responsible_server_ == server)
      {
        return wrapper.q_.id();
      }
    }
    return "";
  }

  RawData RrCatalog::processExisting(const std::string &server,
                                     const std::string &id,
                                     RrQueryBase q)
  {
    std::string request = "";

    std::cout << "processExisting" << std::endl;

    request = findOriginalContainer(id).raw_request_;

    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    if (request.size())
    {
      id_query_map_[request].storeNewId(q.id(), q.origin());
      server_id_map_[server].insert(q.id());
      return id_query_map_[request].raw_query_;
    }

    return "";
  }

  UUID RrCatalog::getInitialId(const std::string &id)
  {
    std::cout << "getInitialId" << std::endl;
    return getOriginQueryId(id);
  }

  RawData RrCatalog::unload(const std::string &server,
                            const std::string &id,
                            bool &unloadable)
  {

    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    auto vec = server_id_map_[server];
    int removed_el_cnt = vec.erase(id);
    server_id_map_[server] = vec;

    std::string query_response = "";

    if (removed_el_cnt)
    {
      std::cout << "unload" << std::endl;
      QueryContainer<RawData> qc = findOriginalContainer(id);

      if (qc.responsible_server_.size())
      {
        query_response = qc.raw_query_;
        qc.removeId(id);

        if (!qc.getIdCount())
        {
          unloadable = true;
          id_query_map_.erase(qc.raw_request_);
        }
        else
        {
          id_query_map_[qc.raw_request_] = qc;
        }
      }
    }

    if (!server_id_map_[server].size())
    {
      server_id_map_.erase(server);
      server_rr_.erase(server);
    }

    return query_response;
  }

  QueryContainer<RawData> RrCatalog::findOriginalContainer(const std::string &id)
  {
    std::cout << "findOriginalContainer: " << id << std::endl;
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    for (auto const &query_entry : id_query_map_)
    {
      if (query_entry.second.rr_ids_.count(id))
      {
        std::cout << "Found!: " << query_entry.second.q_.id() << std::endl;
        return query_entry.second;
      }
    }
    std::cout << "Not found! Returning raw container" << std::endl;
    return QueryContainer<RawData>();
  }

  ServerName RrCatalog::getIdServer(const std::string &id)
  {
    std::cout << "getIdServer: " << id << std::endl;
    return findOriginalContainer(id).responsible_server_;
  }

  std::unordered_map<UUID, std::string> RrCatalog::getAllQueryIds(const std::string &id)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    for (auto const &query_entry : id_query_map_)
    {
      if (query_entry.second.rr_ids_.count(id))
      {
        return query_entry.second.rr_ids_;
      }
    }
    std::unordered_map<UUID, std::string> empty_map;
    return empty_map;
  }

  void RrCatalog::storeDependency(const std::string &query_id,
                                  const std::string &dependency_source,
                                  const std::string &dependency_id)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    id_dependency_map_[query_id].registerDependency(dependency_source, dependency_id);
  }

  std::unordered_map<UUID, std::string> RrCatalog::getDependencies(const std::string &query_id)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    if (id_dependency_map_.count(query_id))
      return id_dependency_map_[query_id].dependencies();

    std::unordered_map<std::string, std::string> empty;
    return empty;
  }

  void RrCatalog::unloadDependency(const std::string &query_id,
                                   const std::string &dependency_id)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    id_dependency_map_[query_id].removeDependency(dependency_id);

    if (!id_dependency_map_[query_id].count())
    {
      id_dependency_map_.erase(query_id);
    }
  }

  UUID RrCatalog::getOriginQueryId(const std::string &query_id)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    for (auto const &dependency_entry : id_dependency_map_)
    {
      if (dependency_entry.second.dependencies().count(query_id))
      {
        return dependency_entry.first;
      }
    }
    return "";
  }

  void RrCatalog::storeClientCallRecord(const std::string &client,
                                        const std::string &id)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    client_id_map_[client].insert(id);
  }

  ClientName RrCatalog::getIdClient(const std::string &id)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    for (auto const &client_queries : client_id_map_)
    {
      if (client_queries.second.count(id))
      {
        return client_queries.first;
      }
    }
    return "";
  }

  std::vector<QueryContainer<RawData>> RrCatalog::getUniqueServerQueries(const std::string &server)
  {
    std::vector<QueryContainer<RawData>> output;
    std::set<UUID> added_messages;

    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    for (auto const &query_id : server_id_map_[server])
    {
      std::cout << "getUniqueServerQueries" << std::endl;
      QueryContainer<RawData> container = findOriginalContainer(query_id);
      if (added_messages.count(container.q_.id()) == 0)
      {
        output.push_back(container);
        added_messages.insert(container.q_.id());
      }
    }

    return output;
  }

  std::set<UUID> RrCatalog::getClientIds(const ClientName &client)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);

    std::cout << "getClientIds for client " << client << std::endl;

    if (client_id_map_.count(client) > 0)
      return client_id_map_[client];

    std::cout << "none found" << std::endl;

    throw ElementNotFoundException(("Client " + client + " not found").c_str());
  }

  std::set<UUID> RrCatalog::getServerIds(const ServerName &server)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    if (server_id_map_.count(server) > 0)
      return server_id_map_[server];

    throw ElementNotFoundException("Server not found");
  }

  void RrCatalog::storeServerRr(const ServerName &server, const RrName &rr)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    server_rr_[server] = rr;
  }

  RrName RrCatalog::getServerRr(const ServerName &server)
  {
    std::lock_guard<std::recursive_mutex> lock(modify_mutex_);
    if (server_rr_.count(server) > 0)
      return server_rr_[server];

    throw ElementNotFoundException("Server not found");
  }

  void RrCatalog::print()
  {
    std::cout << "server_rr_: " << server_rr_.size() << std::endl;
    for (auto const &i : server_rr_)
    {
      std::cout << "{" << std::endl;
      std::cout << i.first << ": ";
      std::cout << i.second << ",";
      std::cout << std::endl;
      std::cout << "}" << std::endl;
    }

    std::cout << "client_id_map_: " << std::endl;
    for (auto const &i : client_id_map_)
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