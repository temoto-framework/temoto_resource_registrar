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

#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_MESSAGE_REGISTRY_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_MESSAGE_REGISTRY_H

#include "rr_query_base.h"
#include "rr_query_request.h"
#include "rr_query_response.h"

#include "glog/logging.h"

#include <algorithm>
#include <boost/functional/hash.hpp>
#include <map>
#include <memory>
#include <unordered_map>

namespace temoto_resource_registrar
{

  class HashFn
  {
  public:
    std::size_t operator()(const RrQueryRequest &r) const;
  };

  class RrMessageRegistry
  {
  public:
    RrMessageRegistry() = default;

    void response(RrQueryBase &query);
    void clearResponses();
    bool hasResponse(RrQueryBase query);
    bool storeResponse(RrQueryRequest req, RrQueryResponse res);

  private:
    void printMapContent();
    std::unordered_map<RrQueryRequest, RrQueryResponse, HashFn> request_response_map_;
  };

  typedef std::shared_ptr<RrMessageRegistry> RrMessageRegistryPtr;
} // namespace temoto_resource_registrar
#endif