#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS1_RESOURCE_REGISTRAR_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS1_RESOURCE_REGISTRAR_H

#include "ros/console.h"
#include "ros/ros.h"

#include "temoto_resource_registrar/rr_base.h"
#include "temoto_resource_registrar/rr_status.h"

#include "temoto_resource_registrar/DataFetchComponent.h"
#include "temoto_resource_registrar/StatusComponent.h"
#include "temoto_resource_registrar/UnloadComponent.h"

#include "rr/message_serializer.h"
#include "rr/ros1_client.h"
#include "rr/ros1_server.h"

#include <functional>

#include "rosconsole_bridge/bridge.h"

REGISTER_ROSCONSOLE_BRIDGE;

namespace temoto_resource_registrar
{

  /**
   * @brief Implementation of the temoto_resource_registrar::RrBase to work on ROS1. 
   * The init() function should be called once ROS is initialized. Else unload and other
   * internal services will not be started up.
   * 
   */
  class ResourceRegistrarRos1 : public RrBase
  {
  public:
    /**
   * @brief Construct a new Resource Registrar Ros 1 object. Take into account that names need to be unique. 
   * They are used as adress fragments for inter-register communications.
   * 
   * @param name 
   */
    ResourceRegistrarRos1(const std::string &name) : RrBase(name)
    {
    }

    ~ResourceRegistrarRos1()
    {
      TEMOTO_INFO_STREAM_("Destroying RR Ros " << name());
      TEMOTO_INFO_STREAM_("unloading clients");
      for (const std::string &client_id : clients_.getIds())
      {
        try
        {
          TEMOTO_INFO_STREAM_("unloading client " << client_id);
          unloadClient(client_id);
        }
        catch (...)
        {
          TEMOTO_WARN_STREAM_("unloading error for client " << client_id);
        }
      }
    }

    /**
     * @brief Startup call for a ResourceRegistrar. Needed to start up support services of the Registrar.
     * 
     */
    void init()
    {
      startServices();
    }

    /**
 * @brief The main call function to request resources from different ResourceRegistrars. Approximate flow of the call is as follows:
 * - Determine if a call is done to a remote RR or to a local object
 * - If it is an internal call:
 * -- Directly access the target RR and call its server, executing appropriate logic
 * - If it is an external call:
 * -- If not done yet create a new client to communicate with the server. Existing cliets are cached.
 * -- Send the users request to the server. This uses the Ros1Client::invoce(Ros1Query<ServiceClass> &wrappedRequest) method
 * -- If the request is unique, the server will execute the user defined loadCallback and return the response. Else a cached response is returned.
 * - Check if a status function was attached to the query. This status function will be called if a status update is sent upstream from the server.
 * 
 * Status functions can be defined for every client. Every client can have only one status function. If it is needed to re-define said function
 * it can be done by raising the overrideStatus parameter. This change applies to all requests done with that client.
 * 
 * @tparam QueryType - Type of the query being executed. It is a normal ROS1 srv.
 * @param rr - Name of the target ResourceRegistrar.
 * @param server - Name of the target Ros1Server.
 * @param query - User query as a QueryType srv object.
 * @param status_func - Optional. User defined status function. Executed when a status is sent by a downstream server.
 * @param override_status - Optional. Forces overwriting of the client status function.
 */
    template <class QueryType>
    void call(const std::string &rr,
              const std::string &server,
              QueryType &query,
              std::function<void(QueryType, Status)> status_func = NULL,
              bool override_status = false)
    {
      TEMOTO_INFO_STREAM_("calling " << rr << " server " << server);
      Ros1Query<QueryType> wrapped_base_query(query);

      privateCall<Ros1Client<QueryType>,
                  Ros1Server<QueryType>,
                  Ros1Query<QueryType>,
                  std::function<void(QueryType, Status)>>(&rr,
                                                          NULL,
                                                          server,
                                                          wrapped_base_query,
                                                          status_func);

      query = wrapped_base_query.rosQuery();

      TEMOTO_INFO_STREAM_("calling " << rr << " server " << server << " done");
    }

    /**
     * @brief A unload call that takes in a adress of the RR that contains the resource and the message ID of the resource that is stored on said RR.
     * If the resource unloaded is the last, the unloadCallback will be executed and the resource will be unloaded from the system. If a new request
     * requires this resource again the loadCallback is executed once again.
     * 
     * @param rr - Name of the target ResourceRegistrar.
     * @param id - ID of the target resource being unloaded
     * @return true 
     * @return false 
     */
    bool unload(const std::string &rr, const std::string &id)
    {
      TEMOTO_INFO_STREAM_("unload Called for rr " << rr << " id: " << id);

      std::string client_name = IDUtils::generateUnload(rr);
      initClient<UnloadComponent>(client_name, unload_clients_);

      temoto_resource_registrar::UnloadComponent unload_srv;
      unload_srv.request.target = id;

      bool res = unload_clients_[client_name]->call(unload_srv);
      TEMOTO_INFO_STREAM_("result: " << res);

      return res;
    }

    /**
 * @brief Metod used to deserialize a message from  the catalog for status propagation. Messages are not held on the client
 * side and therefore need to be sent to the client. The client also needs to be able to deserialize them. This method is
 * used for that.
 * 
 * @tparam QueryType 
 * @param container 
 * @return QueryType 
 */
    template <class QueryType>
    QueryType deSerializeQuery(const QueryContainer<RawData> &container)
    {
      TEMOTO_INFO_STREAM_("deSerializeBaseQuery ROS ");
      QueryType q;

      q.request = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename QueryType::Request>(container.raw_request_);
      q.response = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename QueryType::Response>(container.raw_query_);

      q.response.temoto_metadata.request_id = container.q_.id();

      return q;
    }

    template <class ClientQueryType>
    void registerClientCallback(const std::string &rr_name,
                                const std::string &server_name,
                                const std::string &query_id,
                                std::function<void(ClientQueryType, Status)> user_callback)
    {
      std::string client_id = createClient<Ros1Client<ClientQueryType>>(rr_name, server_name);
      storeClientQueryStatusCb<Ros1Client<ClientQueryType>, std::function<void(ClientQueryType, Status)>>(client_id, query_id, user_callback);
    }

    void registerDependency(const std::string &rr_name,
                            const std::string &query_id,
                            const std::string &parent_query_id)
    {
      rr_catalog_->storeDependency(parent_query_id, rr_name, query_id);
      autoSaveCatalog();
    }

    /**
 * @brief Get the Ros Child Queries object. Used to get executed queries of a dependency of the query
 * defined. Takes in an query ID and the serverName the dependency used.
 * 
 * @tparam QueryType - type of the query. Used for deserialization
 * @param id - id of the query being investigated. Used for dependency determination
 * @param server_name - name of the resource server being fetched. Many servers with the same resource
 * type can exist, this is why a name needs to be defined.
 * @return std::map<std::string, QueryType> 
 */
    template <class QueryType>
    std::map<std::string, QueryType> getRosChildQueries(const std::string &id, const std::string &server_name)
    {
      TEMOTO_DEBUG_STREAM_("getting RR queries for id " << id << " and server " << server_name);
      std::map<std::string, std::pair<std::string, std::string>> serialised_queries = getChildQueries(id, server_name);

      TEMOTO_DEBUG_STREAM_("Found " << serialised_queries.size() << " queries");
      std::map<std::string, QueryType> ros_queries;

      TEMOTO_DEBUG_STREAM_("Deserializing strings to query type");
      for (const auto &el : serialised_queries)
      {
        QueryType q;
        q.request = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename QueryType::Request>(el.second.first);
        q.response = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename QueryType::Response>(el.second.second);
        ros_queries[el.first] = q;
      }
      TEMOTO_DEBUG_STREAM_("Returning...");
      return ros_queries;
    }

    template <class QueryType>
    std::vector<QueryType> getServerQueries(const std::string &server)
    {
      TEMOTO_INFO_STREAM_("getServerQueries printing before processign catalog.");
      std::string server_name = IDUtils::generateServerName(name(), server);
      rr_catalog_->print();
      std::vector<QueryType> out;

      for (auto const &queryContainer : rr_catalog_->getUniqueServerQueries(server_name))
      {
        out.push_back(deSerializeQuery<QueryType>(queryContainer));
      }

      TEMOTO_INFO_STREAM_("getServerQueries printing after processign catalog.");
      rr_catalog_->print();

      return out;
    }

    template <class QueryType>
    void updateQueryResponse(const std::string &server,
                             const QueryType &call)
    {
      TEMOTO_INFO_STREAM_("updateQueryResponse for server " << server);

      std::string serialized_request = temoto_resource_registrar::MessageSerializer::serializeMessage<typename QueryType::Request>(sanityzeData<typename QueryType::Request>(call.request));
      std::string serialized_response = temoto_resource_registrar::MessageSerializer::serializeMessage<typename QueryType::Response>(sanityzeData<typename QueryType::Response>(call.response));
      updateQuery(server, serialized_request, serialized_response);
    }

  protected:
    std::unordered_map<std::string, std::unique_ptr<ros::ServiceClient>> unload_clients_;
    std::unordered_map<std::string, std::unique_ptr<ros::ServiceClient>> status_clients_;
    std::unordered_map<std::string, std::unique_ptr<ros::ServiceClient>> fetch_clients_;

    /**
 * @brief Virtual method that needs to be implemented on every extension of RR_CORE. This method creates and sends the status
 * callback to a target that is defined in the statusData.
 * 
 * @param target_rr 
 * @param request_id 
 * @param status_srv 
 * @return true 
 * @return false 
 */
    virtual bool callStatusClient(const std::string &target_rr, const std::string &request_id, Status status_data)
    {

      std::string client_name = IDUtils::generateStatus(target_rr);

      initClient<StatusComponent>(client_name, status_clients_);

      temoto_resource_registrar::StatusComponent status_srv;
      TEMOTO_INFO_STREAM_("callStatusClient");
      auto container = rr_catalog_->findOriginalContainer(status_data.id_);
      if (!container.empty_)
      {
        ROS_WARN_STREAM("CONTAINER EXISTS");
        status_srv.request.serialised_request = container.raw_request_;
        status_srv.request.serialised_response = container.raw_query_;

        status_data.serialised_request_ = container.raw_request_;
        status_data.serialised_response_ = container.raw_query_;
      }
      else
      {
        ROS_WARN_STREAM("CONTAINER does not EXISTS");
      }

      handleRrServerCb(request_id, status_data);

      status_srv.request.target = status_data.id_;
      status_srv.request.status = static_cast<int>(status_data.state_);
      status_srv.request.message = status_data.message_;

      TEMOTO_INFO_STREAM_("calling status client " << client_name << " target id: " << status_data.id_);

      return status_clients_[client_name]->call(status_srv);
    };

    virtual std::map<std::string, std::pair<std::string, std::string>> callDataFetchClient(const std::string &target_rr,
                                                                                           const std::string &origin_rr,
                                                                                           const std::string &server_name)
    {
      TEMOTO_INFO_STREAM_("callDataFetchClient");

      std::string client_name = IDUtils::generateFetch(target_rr);
      initClient<DataFetchComponent>(client_name, fetch_clients_);

      temoto_resource_registrar::DataFetchComponent data_fetch_srv;

      data_fetch_srv.request.origin_rr = origin_rr;
      data_fetch_srv.request.server_name = server_name;

      TEMOTO_INFO_STREAM_("calling data fetch client " << client_name << " - " << data_fetch_srv.request);

      fetch_clients_[client_name]->call(data_fetch_srv);

      std::map<std::string, std::pair<std::string, std::string>> res;

      int c = 0;
      for (const auto &id : data_fetch_srv.response.ids)
      {
        std::pair<std::string, std::string> req_res(data_fetch_srv.response.serialized_requests.at(c),
                                                    data_fetch_srv.response.serialized_responses.at(c));
        res[id] = req_res;
        c++;
      }

      return res;
    }

    /**
 * @brief ROS 1 implementation of the unloadResource method. 
 * 
 * @param id 
 * @param dependency 
 */
    virtual void unloadResource(const std::string &id, const std::pair<const std::string, std::string> &dependency)
    {
      bool unload_status = unload(dependency.second, dependency.first);
      if (unload_status)
      {
        rr_catalog_->unloadDependency(id, dependency.first);
        autoSaveCatalog();
      }
    }

  private:
    ros::ServiceServer unload_service_;
    ros::ServiceServer status_service_;
    ros::ServiceServer fetch_service_;

    template <class ServiceClass>
    void initClient(const std::string &client_name, std::unordered_map<std::string, std::unique_ptr<ros::ServiceClient>> &client_map)
    {
      ros::NodeHandle nh;

      if (client_map.count(client_name) == 0)
      {
        TEMOTO_INFO_STREAM_("creating client " << client_name);
        auto sc = nh.serviceClient<ServiceClass>(client_name);
        auto client = std::make_unique<ros::ServiceClient>(sc);
        client_map[client_name] = std::move(client);
        TEMOTO_INFO_STREAM_("created client...");
      }
    }

    void startServices()
    {
      TEMOTO_INFO_STREAM_("Starting up services...");
      ros::NodeHandle nh;
      TEMOTO_INFO_STREAM_("Starting unload_service_... " << IDUtils::generateUnload(name()));
      unload_service_ = nh.advertiseService(IDUtils::generateUnload(name()), &ResourceRegistrarRos1::unloadCallback, this);
      TEMOTO_INFO_STREAM_("Starting status_service_... " << IDUtils::generateStatus(name()));
      status_service_ = nh.advertiseService(IDUtils::generateStatus(name()), &ResourceRegistrarRos1::statusCallback, this);
      TEMOTO_INFO_STREAM_("Starting fetch_service_... " << IDUtils::generateFetch(name()));
      fetch_service_ = nh.advertiseService(IDUtils::generateFetch(name()), &ResourceRegistrarRos1::dataFetchCallback, this);
    }

    bool unloadCallback(UnloadComponent::Request &req, UnloadComponent::Response &res)
    {
      TEMOTO_INFO_STREAM_("printing catalog before unload callback");
      rr_catalog_->print();

      TEMOTO_INFO_STREAM_("unloadCallback " << req.target);
      std::string id = req.target;
      TEMOTO_INFO_STREAM_("std::string id " << id);
      res.status = localUnload(id);
      return true;
    }

    bool statusCallback(StatusComponent::Request &req, StatusComponent::Response &res)
    {
      TEMOTO_INFO_STREAM_("statusCallback: " << req.target);
      handleStatus(req.target, {static_cast<Status::State>(req.status), req.target, req.message, req.serialised_request, req.serialised_response});
      return true;
    }

    bool dataFetchCallback(DataFetchComponent::Request &req, DataFetchComponent::Response &res)
    {
      TEMOTO_INFO_STREAM_("syncCallback " << req);
      std::map<UUID, std::pair<std::string, std::string>> resMap = handleDataFetch(req.origin_rr, req.server_name);
      std::vector<std::string> ids, serialized_requests, serialized_responses;

      for (const auto &el : resMap)
      {
        ids.push_back(el.first);
        serialized_requests.push_back(el.second.first);
        serialized_responses.push_back(el.second.second);
      }

      res.ids = ids;
      res.serialized_requests = serialized_requests;
      res.serialized_responses = serialized_responses;
      return true;
    }

    /**
 * @brief Method used to strip system metadata from a message. This is useful when message uniqueness is required.
 * 
 * @tparam MsgClass 
 * @param data 
 * @return MsgClass 
 */
    template <class MsgClass>
    static MsgClass sanityzeData(MsgClass data)
    {
      MsgClass empty;
      data.temoto_metadata = empty.temoto_metadata;
      return data;
    }
  };
}
#endif