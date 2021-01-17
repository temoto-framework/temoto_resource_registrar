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

#include "temoto_resource_registrar/rr_connection_registry.h"

namespace temoto_resource_registrar
{

  /**
 * 
 * RrServerRepository
 * 
 */

  bool RrServerRepository::add(std::unique_ptr<RrServerBase> server)
  {
    auto ret = rr_servers_.insert(std::make_pair(server->id(), std::move(server)));
    return ret.second;
  };

  bool RrServerRepository::remove(const std::string &id)
  {
    return rr_servers_.erase(id) > 0;
  };

  bool RrServerRepository::exists(const std::string &id)
  {
    auto it = rr_servers_.find(id);
    if (it != rr_servers_.end())
    {
      return true;
    }
    return false;
  };

  RrServerBase *RrServerRepository::get(const std::string &id)
  {
    auto it = rr_servers_.find(id);
    if (it != rr_servers_.end())
    {
      return it->second.get();
    }
    throw(ServerNotFoundException("Could not find server: " + id));
  };

  std::vector<std::string> RrServerRepository::getIds()
  {
    std::vector<std::string> ids;
    for (auto it = rr_servers_.begin(); it != rr_servers_.end(); ++it)
    {
      ids.push_back(it->first);
    }
    return ids;
  };

  /**
 * 
 * RrClientRepository
 * 
 */

  bool RrClientRepository::add(std::unique_ptr<RrClientBase> client)
  {
    auto ret = rr_clients_.insert(std::make_pair(client->id(), std::make_unique<RrClientRepostioryEntry>(std::move(client))));
    return ret.second;
  };

  bool RrClientRepository::remove(const std::string &id)
  {
    return rr_clients_.erase(id) > 0;
  };

  bool RrClientRepository::exists(const std::string &id)
  {
    return rr_clients_.count(id) > 0;
  };

  /**
 * 
 * RrClientRepostioryEntry
 * 
 */

  RrClientRepostioryEntry::RrClientRepostioryEntry(std::unique_ptr<RrClientBase> client)
      : client_pointer_(std::move(client)){};

  std::string RrClientRepostioryEntry::id()
  {
    return client_pointer_->id();
  };

  /**
 * 
 * RrConnectionRegistry
 * 
 */

  bool RrConnectionRegistry::addServer(std::unique_ptr<RrServerBase> server)
  {
    server_repository_.add(std::move(server));
  };

  bool RrConnectionRegistry::addClient(std::unique_ptr<RrClientBase> client)
  {
    client_repository_.add(std::move(client));
  };

  bool RrConnectionRegistry::hasClient(std::string const &id)
  {
    return client_repository_.exists(id);
  };

  bool RrConnectionRegistry::hasServer(std::string const &id)
  {
    return server_repository_.exists(id);
  };

  RrServerBase *RrConnectionRegistry::fetchServer(std::string const &id)
  {
    return server_repository_.get(id);
  };

  std::vector<std::string> RrConnectionRegistry::registeredServers()
  {
    return server_repository_.getIds();
  };

} // namespace temoto_resource_registrar
