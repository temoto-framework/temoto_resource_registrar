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

#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_CATALOG_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_CATALOG_H

#include "rr_query_base.h"
#include "rr_query_container.h"

#include <algorithm>
#include <boost/serialization/access.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

namespace temoto_resource_registrar
{
  using RawData = std::string;
  using ServerName = std::string;
  using ClientName = std::string;
  using UUID = std::string;

  class DependencyContainer
  {
  public:
    DependencyContainer() {}
    DependencyContainer(const std::string &rr,
                        const std::string &id)
    {
      registerDependency(rr, id);
    }

    void registerDependency(const std::string &rr, const std::string &id) { id_rr_map_[id] = rr; }

    void removeDependency(const std::string &id) { id_rr_map_.erase(id); }

    int count() { return id_rr_map_.size(); }

    void print() const
    {
      std::cout << "{";
      for (auto const &i : id_rr_map_)
      {

        std::cout << i.first << ": " << i.second << ", ";
      }
      std::cout << "}" << std::endl;
    }

    std::unordered_map<std::string, std::string> dependencies() const { return id_rr_map_; }

  protected:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* version */)
    {
      ar &id_rr_map_;
    }

  private:
    std::unordered_map<std::string, std::string> id_rr_map_;
  };

  class RrCatalog
  {
  public:
    RrCatalog() = default;

    void storeQuery(const ServerName &server, RrQueryBase q, RawData reqData, RawData qData);
    void updateResponse(const ServerName &server, RawData request, RawData response);
    UUID queryExists(const ServerName &server, RawData qData);
    RawData processExisting(const ServerName &server, const UUID &id, RrQueryBase q);
    UUID getInitialId(const UUID &id);

    RawData unload(const ServerName &server, const UUID &id, bool &unloadable);

    ServerName getIdServer(const UUID &id);
    std::unordered_map<UUID, std::string> getAllQueryIds(const std::string &id);

    std::unordered_map<UUID, std::string> getDependencies(const std::string &queryId);
    void storeDependency(const UUID &queryId, const ServerName &dependencySource, const UUID &dependencyId);
    void unloadDependency(const UUID &queryId, const UUID &dependencyId);
    UUID getOriginQueryId(const UUID &queryId);

    QueryContainer<RawData> findOriginalContainer(const UUID &id);

    void storeClientCallRecord(const ClientName &client, const UUID &id);
    ClientName getIdClient(const UUID &id);

    std::vector<QueryContainer<RawData>> getUniqueServerQueries(const ServerName &server);

    void print();

  protected:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* version */)
    {
      ar &server_id_map_ &client_id_map_ &id_query_map_ &id_dependency_map_;
    }

  private:
    std::unordered_map<ClientName, std::set<UUID>> client_id_map_;
    std::unordered_map<ServerName, std::set<UUID>> server_id_map_;
    std::unordered_map<RawData, QueryContainer<RawData>> id_query_map_;
    std::unordered_map<UUID, DependencyContainer> id_dependency_map_;
  };

  typedef std::shared_ptr<RrCatalog> RrCatalogPtr;
} // namespace temoto_resource_registrar
#endif