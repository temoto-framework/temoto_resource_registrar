#include "temoto_resource_registrar/rr_registry.h"

namespace temoto_resource_registrar
{

  /**
 * 
 * 
 * RrServerRepository
 * 
 * 
 */

  bool RrServerRepository::add(std::unique_ptr<RrServerBase> server)
  {
    auto ret = rr_servers_.insert(std::make_pair(server->id(), std::move(server)));
    return ret.second;
  };

  bool RrServerRepository::remove(const std::string &id){
      //TODO: fill out
  };

  bool RrServerRepository::exists(const std::string &id)
  {
    return rr_servers_.count(id) > 0;
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
 * 
 * RrClientRepository
 * 
 * 
 */

  bool RrClientRepository::add(std::unique_ptr<RrClientBase> client)
  {
    auto ret = rr_clients_.insert(std::make_pair(client->id(), std::make_unique<RrClientRepostioryEntry>(std::move(client))));
    return ret.second;
  };

  bool RrClientRepository::remove(const std::string &id){
      //TODO: fill out
  };

  bool RrClientRepository::exists(const std::string &id)
  {
    return rr_clients_.count(id) > 0;
  };

  void RrClientRepository::registerStatusCallback(const std::string &id, std::function<void(RrClientBase)> rrStatusCallback){};

  /**
 * 
 * 
 * RrClientRepostioryEntry
 * 
 * 
 */

  RrClientRepostioryEntry::RrClientRepostioryEntry(std::unique_ptr<RrClientBase> client)
      : client_pointer_(std::move(client)){};

  std::string RrClientRepostioryEntry::id(){

  };

  void RrClientRepostioryEntry::executeStatusCallback(){

  };

  /**
 * 
 * 
 * RrRegistry
 * 
 * 
 */

  bool RrRegistry::addServer(std::unique_ptr<RrServerBase> server)
  {
    server_repository_.add(std::move(server));
  };

  bool RrRegistry::addClient(std::unique_ptr<RrClientBase> client)
  {
    client_repository_.add(std::move(client));
  };

  bool RrRegistry::hasClient(std::string const &id)
  {
    return client_repository_.exists(id);
  };

  bool RrRegistry::hasServer(std::string const &id)
  {
    return server_repository_.exists(id);
  };

  RrServerBase *RrRegistry::fetchServer(std::string const &id)
  {
    return server_repository_.get(id);
  };

  std::vector<std::string> RrRegistry::registeredServers()
  {
    return server_repository_.getIds();
  };

} // namespace temoto_resource_registrar
