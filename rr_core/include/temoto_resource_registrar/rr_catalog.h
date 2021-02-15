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
#include <set>
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
                                                responsibleServer_(server)
    {
      storeNewId(q.id());
    };

    void storeNewId(const std::string &id)
    {
      ids_.insert(id);
    }

    void removeId(const std::string &id)
    {
      ids_.erase(id);
    }

    RawData rawQuery_;
    RawData rawRequest_;
    RrQueryBase q_;
    std::set<std::string> ids_;
    std::string responsibleServer_;

  protected:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* version */)
    {
      ar &q_ &rawRequest_ &rawQuery_ &ids_ &responsibleServer_;
    }
  };

  class DependencyContainer
  {
  public:
    DependencyContainer() {}
    DependencyContainer(const std::string &rr,
                        const std::string &id)
    {
      registerDependency(rr, id);
    }

    void registerDependency(const std::string &rr, const std::string &id)
    {
      id_rr_map_[id] = rr;
    }

    void print() const
    {
      std::cout << "{";
      for (auto const &i : id_rr_map_)
      {

        std::cout << i.first << ": " << i.second << ", ";
      }
      std::cout << "}" << std::endl;
    }

    std::unordered_map<std::string, std::string> dependencies()
    {
      return id_rr_map_;
    }

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

    void storeQuery(const std::string &server, RrQueryBase q, RawData reqData, RawData qData);
    std::string queryExists(const std::string &server, RawData qData);
    RawData processExisting(const std::string &server, const std::string &id, RrQueryBase q);
    RawData unload(const std::string &server, const std::string &id);
    bool canBeUnloaded(const std::string &server);
    std::string getIdServer(const std::string &id);
    void storeDependency(const std::string &queryId, const std::string &dependencySource, const std::string &dependencyId);
    std::unordered_map<std::string, std::string> getDependencies(const std::string &queryId);

    void print();

  protected:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* version */);

  private:
    std::unordered_map<std::string, std::set<std::string>> server_id_map_;
    std::unordered_map<RawData, QueryContainer> id_query_map_;

    std::unordered_map<std::string, DependencyContainer> id_dependency_map;

    QueryContainer findOriginalContainer(const std::string &id);
  };

  typedef std::shared_ptr<RrCatalog> RrCatalogPtr;
} // namespace temoto_resource_registrar
#endif