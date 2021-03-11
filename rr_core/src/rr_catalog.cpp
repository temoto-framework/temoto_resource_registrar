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
    print();
    std::cout << "in store query..." << std::endl;

    id_query_map_[reqData] = QueryContainer<RawData>(q, reqData, qData, server);
    server_id_map_[server].insert(q.id());

    std::cout << "storage done..." << std::endl;
    print();
  }

  std::string RrCatalog::queryExists(const std::string &server, RawData reqData)
  {
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

    request = findOriginalContainer(id).rawRequest_;

    if (request.size())
    {
      id_query_map_[request].storeNewId(q.id(), q.origin());
      server_id_map_[server].insert(q.id());
      return id_query_map_[request].rawQuery_;
    }

    return "";
  }

  std::string RrCatalog::getInitialId(const std::string &id)
  {
    //QueryContainer cotnainer = findOriginalContainer(id);
    //std::cout << cotnainer.getIdCount() << std::endl;
    //return cotnainer.q_.id();
    return getOriginQueryId(id);
  }

  RawData RrCatalog::unload(const std::string &server, const std::string &id)
  {

    auto vec = server_id_map_[server];
    int removedElCnt = vec.erase(id);
    server_id_map_[server] = vec;

    std::string queryResp = "";

    if (removedElCnt)
    {
      QueryContainer<RawData> qc = findOriginalContainer(id);

      if (qc.responsibleServer_.size())
      {
        queryResp = qc.rawQuery_;
        qc.removeId(id);

        if (!qc.getIdCount())
        {
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

  bool RrCatalog::canBeUnloaded(const std::string &server)
  {
    int serverCount = server_id_map_.count(server);

    if (!serverCount)
    {
      return true;
    }

    return server_id_map_[server].size() == 0;
  }

  QueryContainer<RawData> RrCatalog::findOriginalContainer(const std::string &id)
  {
    for (auto const &queryEntry : id_query_map_)
    {
      if (queryEntry.second.rr_ids_.count(id))
      {
        return queryEntry.second;
      }
    }

    return QueryContainer<RawData>();
  }

  std::string RrCatalog::getIdServer(const std::string &id)
  {
    std::cout << "getIdServer: " << id << std::endl;
    return findOriginalContainer(id).responsibleServer_;
  }

  std::unordered_map<std::string, std::string> RrCatalog::getAllQueryIds(const std::string &id)
  {
    for (auto const &queryEntry : id_query_map_)
    {
      if (queryEntry.second.rr_ids_.count(id))
      {
        return queryEntry.second.rr_ids_;
      }
    }
    std::unordered_map<std::string, std::string> emptyMap;
    return emptyMap;
  }

  void RrCatalog::storeDependency(const std::string &queryId, const std::string &dependencySource, const std::string &dependencyId)
  {
    id_dependency_map_[queryId].registerDependency(dependencySource, dependencyId);
  }

  std::unordered_map<std::string, std::string> RrCatalog::getDependencies(const std::string &queryId)
  {
    if (id_dependency_map_.count(queryId))
      return id_dependency_map_[queryId].dependencies();

    std::unordered_map<std::string, std::string> empty;
    return empty;
  }

  void RrCatalog::unloadDependency(const std::string &queryId, const std::string &dependencyId)
  {
    id_dependency_map_[queryId].removeDependency(dependencyId);

    if (!id_dependency_map_[queryId].count())
    {
      id_dependency_map_.erase(queryId);
    }
  }

  std::string RrCatalog::getOriginQueryId(const std::string &queryId)
  {
    for (auto const &dependencyEntry : id_dependency_map_)
    {
      if (dependencyEntry.second.dependencies().count(queryId))
      {
        return dependencyEntry.first;
      }
    }
    return "";
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

  template <class Archive>
  void RrCatalog::serialize(Archive &ar, const unsigned int /* version */)
  {
    ar &server_id_map_ &id_query_map_ &id_dependency_map_;
  }

} // namespace temoto_resource_registrar