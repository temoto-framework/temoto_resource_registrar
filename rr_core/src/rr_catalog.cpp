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
  void RrCatalog::clearResponses()
  {
    request_response_map_.clear();
  }

  void RrCatalog::processResponse(RawData req, RawData res)
  {
    auto it = request_response_map_.find(req);
    if (it != request_response_map_.end())
    {
    }
    else
    {
      storeResponse(req, res);
    }
  }

  bool RrCatalog::hasResponse(RawData queryData)
  {
    return request_response_map_.count(queryData) > 0;
  }

  bool RrCatalog::storeResponse(RawData req, RawData res)
  {
    auto ret = request_response_map_.insert(std::make_pair(req, res));
    return ret.second;
  }

  void RrCatalog::storeDependency(std::string dependent, std::string dependency)
  {
    if (request_dependency_map_.find(dependent) != request_dependency_map_.end())
    {
      request_dependency_map_.at(dependent).push_back(dependency);
    }
    else
    {
      request_dependency_map_.insert({dependent, std::vector<std::string>{dependency}});
    }
  }

  RawData RrCatalog::fetchFromStorage(RawData req)
  {
    if (hasResponse(req))
    {
      return fetchResponse(req);
    }
  }

  RawData RrCatalog::fetchResponse(RawData req)
  {
    auto it = request_response_map_.find(req);
    if (it != request_response_map_.end())
    {
      return it->second;
    }
  }

  void RrCatalog::storeServerQuery(const std::string &server, const std::string &messageId, RawData request, RawData query)
  {
    server_query_map_[server].insert(std::make_pair(request, query));
    id_request_map_.insert(std::make_pair(messageId, request));
  }

  bool RrCatalog::hasStoredServerQuery(const std::string &server, RawData request)
  {
    return server_query_map_[server].count(request) > 0;
  }

  RawData RrCatalog::fetchFromServerStorage(const std::string &server, RawData req)
  {
    return server_query_map_[server][req];
  }

  bool RrCatalog::removeServerQuery(const std::string &server, const std::string &id)
  {
    auto serverRequests = server_query_map_[server];
    int currentSize = serverRequests.size();
    serverRequests.erase(id_request_map_[id]);

    return currentSize > serverRequests.size();
  }

} // namespace temoto_resource_registrar