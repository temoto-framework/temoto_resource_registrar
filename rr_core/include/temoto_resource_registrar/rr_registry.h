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

#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_REGISTRY_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_REGISTRY_H

#include "rr_client_base.h"
#include "rr_server_base.h"
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

namespace temoto_resource_registrar
{

  class ServerNotFoundException
  {
  public:
    ServerNotFoundException(const std::string &msg) : msg_(msg) {}
    ~ServerNotFoundException() {}
    std::string getMessage() const { return (msg_); }

  private:
    std::string msg_;
  };

  class RrServerRepository
  {
  public:
    bool add(std::unique_ptr<RrServerBase> server);
    bool remove(const std::string &id);
    bool exists(const std::string &id);
    RrServerBase *get(const std::string &id);
    std::vector<std::string> getIds();

  private:
    std::unordered_map<std::string, std::unique_ptr<RrServerBase>> rr_servers_;
  };

  class RrClientRepostioryEntry
  {
  public:
    RrClientRepostioryEntry();
    RrClientRepostioryEntry(std::unique_ptr<RrClientBase> client);
    std::string id();
    void executeStatusCallback();

  private:
    std::unique_ptr<RrClientBase> client_pointer_;
    std::function<void(RrClientBase)> status_callback_;
  };

  class RrClientRepository
  {
  public:
    bool add(std::unique_ptr<RrClientBase> client);
    bool remove(const std::string &id);
    bool exists(const std::string &id);
    void registerStatusCallback(std::function<void(RrClientBase)> rrStatusCallback);

  private:
    std::unordered_map<std::string, std::unique_ptr<RrClientRepostioryEntry>> rr_clients_;
    std::vector<std::function<void(RrClientBase)>> client_status_callbacks_;
  };

  class RrRegistry
  {
  public:
    RrRegistry() = default;

    bool addServer(std::unique_ptr<RrServerBase> server);
    bool addClient(std::unique_ptr<RrClientBase> client);
    void registerStatusCallback(std::function<void(RrClientBase)> rrStatusCallback);

    bool hasClient(std::string const &id);
    bool hasServer(std::string const &id);

    RrServerBase *fetchServer(std::string const &id);
    std::vector<std::string> registeredServers();

  private:
    RrServerRepository server_repository_;
    RrClientRepository client_repository_;
  };

  typedef std::shared_ptr<RrRegistry> RrRegistryPtr;

} // namespace temoto_resource_registrar

#endif