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

#include <algorithm>
#include <boost/functional/hash.hpp>
#include <map>
#include <memory>
#include <unordered_map>

namespace temoto_resource_registrar
{
  using RawData = std::string;

  class QueryContainer
  {
  public:
    QueryContainer(){};
    QueryContainer(RrQueryBase q,
                   RawData req,
                   RawData data,
                   const std::string &server) : q_(q),
                                                rawRequest_(req),
                                                rawQuery_(data),
                                                responsibleServer_(server){};

    void storeNewId(const std::string &id)
    {
      ids_.push_back(id);
    }

    RawData rawQuery_;
    RawData rawRequest_;
    RrQueryBase q_;
    std::vector<std::string> ids_;
    std::string responsibleServer_;

  protected:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* version */)
    {
      ar &q_ &rawRequest_ &rawQuery_ &ids_ &responsibleServer_;
    }
  };

  class RrCatalog
  {
  public:
    RrCatalog() = default;

    void storeQuery(const std::string &server, RrQueryBase q, RawData reqData, RawData qData);
    std::string queryExists(const std::string &server, RawData qData);
    RawData processExisting(const std::string &server, const std::string &id, RrQueryBase q);
    RawData unload(const std::string &server, const std::string &id);

  private:
    std::unordered_map<std::string, std::vector<std::string>> server_id_map_;
    std::unordered_map<std::string, QueryContainer> id_query_map_;
  };

  typedef std::shared_ptr<RrCatalog> RrCatalogPtr;
} // namespace temoto_resource_registrar
#endif