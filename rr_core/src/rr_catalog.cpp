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
    id_query_map_[q.id()] = QueryContainer(q, reqData, qData, server);
    server_id_map_[server].push_back(q.id());
  }

  std::string RrCatalog::queryExists(const std::string &server, RawData reqData)
  {
    for (auto const &query : id_query_map_)
    {
      QueryContainer wrapper = query.second;
      if (wrapper.rawRequest_ == reqData && wrapper.responsibleServer_ == server)
      {
        return query.second.q_.id();
      }
    }
    return "";
  }

  RawData RrCatalog::processExisting(const std::string &server, const std::string &id, RrQueryBase q)
  {
    id_query_map_[id].storeNewId(q.id());
    server_id_map_[server].push_back(q.id());

    return id_query_map_[id].rawQuery_;
  }

  RawData RrCatalog::unload(const std::string &server, const std::string &id)
  {
    auto vec = server_id_map_[server];

    std::remove(vec.begin(), vec.end(), id);

    for (auto const &query : id_query_map_)
    {
    }

    std::cout << "AAAAAAAAAAAA "
              << "\n";
    return "";
  }

} // namespace temoto_resource_registrar