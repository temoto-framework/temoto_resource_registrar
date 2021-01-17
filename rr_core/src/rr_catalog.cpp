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
  };

  void RrCatalog::respond(RrQueryBase &query)
  {
    auto it = request_response_map_.find(query.request());
    if (it != request_response_map_.end())
    {
      query.updateResponse(it->second);
    }
    else
    {
      storeResponse(query.request(), query.response());
    }
  };

  bool RrCatalog::hasResponse(RrQueryBase query)
  {
    return request_response_map_.count(query.request()) > 0;
  };

  bool RrCatalog::storeResponse(RrQueryRequest req, RrQueryResponse res)
  {
    auto ret = request_response_map_.insert(std::make_pair(req, res));
    return ret.second;
  };

  std::size_t HashFn::operator()(const RrQueryRequest &r) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, r.message_);
    return seed;
  };

} // namespace temoto_resource_registrar