#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS1_RESOURCE_REGISTRAR_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS1_RESOURCE_REGISTRAR_H

#include "rclcpp/rclcpp.hpp"

#include "temoto_resource_registrar/rr_base.h"
#include "temoto_resource_registrar/rr_status.h"

#include "tutorial_interfaces/srv/data_fetch_component.hpp"
#include "tutorial_interfaces/srv/status_component.hpp"
#include "tutorial_interfaces/srv/unload_component.hpp"

#include "rr/message_serializer.h"
#include "rr/ros2_client.h"
#include "rr/ros2_server.h"

#include <functional>

using rclcpp::executors::MultiThreadedExecutor;

namespace temoto_resource_registrar
{

  /**
   * @brief Implementation of the temoto_resource_registrar::RrBase to work on ROS1. 
   * The init() function should be called once ROS is initialized. Else unload and other
   * internal services will not be started up.
   * 
   */
  class ResourceRegistrarRos2 : public RrBase
  {
  public:
    /**
   * @brief Construct a new Resource Registrar Ros 1 object. Take into account that names need to be unique. 
   * They are used as adress fragments for inter-register communications.
   * 
   * @param name 
   */
    ResourceRegistrarRos2(const std::string &name) : RrBase(name)
    {
    }

    ~ResourceRegistrarRos2()
    {
      //ROS_INFO_STREAM("Destroying RR Ros " << name());
      //ROS_INFO_STREAM("unloading clients");
      for (const std::string &client_id : clients_.getIds())
      {
        try
        {
          //ROS_INFO_STREAM("unloading client " << client_id);
          unloadClient(client_id);
        }
        catch (...)
        {
          //ROS_WARN_STREAM("unloading error for client " << client_id);
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
      //ROS_INFO_STREAM("calling " << rr << " server " << server);
      Ros2Query<QueryType> wrapped_base_query(query);

      privateCall<Ros2Client<QueryType>,
                  Ros2Server<QueryType>,
                  Ros2Query<QueryType>,
                  std::function<void(QueryType, Status)>>(&rr,
                                                          NULL,
                                                          server,
                                                          wrapped_base_query,
                                                          status_func,
                                                          override_status);

      query = wrapped_base_query.rosQuery();

      //ROS_INFO_STREAM("calling " << rr << " server " << server << " done");
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
      //ROS_INFO_STREAM("unload Called for rr " << rr << " id: " << id);

      std::string client_name = IDUtils::generateUnload(rr);
      initClient<tutorial_interfaces::srv::UnloadComponent>(client_name, unload_clients_);

      auto request = std::make_shared<tutorial_interfaces::srv::UnloadComponent::Request>();
      request->target = id;

      auto result = unload_clients_[client_name]->async_send_request(request);
      return rclcpp::spin_until_future_complete(node_, result) == rclcpp::executor::FutureReturnCode::SUCCESS;

      //bool res = unload_clients_[client_name]->call(unload_srv);
      //ROS_INFO_STREAM("result: " << res);

      //return res;
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
      //ROS_INFO_STREAM("deSerializeBaseQuery ROS ");
      QueryType q;

      q.request = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename QueryType::Request>(container.raw_request_);
      q.response = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename QueryType::Response>(container.raw_query_);

      q.response.temoto_metadata.request_id = container.q_.id();

      return q;
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
      //ROS_DEBUG_STREAM("getting RR queries for id " << id << " and server " << server_name);
      std::map<std::string, std::pair<std::string, std::string>> serialised_queries = getChildQueries(id, server_name);

      //ROS_DEBUG_STREAM("Found " << serialised_queries.size() << " queries");
      std::map<std::string, QueryType> ros_queries;

      //ROS_DEBUG_STREAM("Deserializing strings to query type");
      for (const auto &el : serialised_queries)
      {
        QueryType q;
        q.request = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename QueryType::Request>(el.second.first);
        q.response = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename QueryType::Response>(el.second.second);
        ros_queries[el.first] = q;
      }
      //ROS_DEBUG_STREAM("Returning...");
      return ros_queries;
    }

    template <class QueryType>
    std::vector<QueryType> getServerQueries(const std::string &server)
    {
      //ROS_INFO_STREAM("getServerQueries printing before processign catalog.");
      std::string server_name = IDUtils::generateServerName(name(), server);
      rr_catalog_->print();
      std::vector<QueryType> out;

      for (auto const &queryContainer : rr_catalog_->getUniqueServerQueries(server_name))
      {
        out.push_back(deSerializeQuery<QueryType>(queryContainer));
      }

      //ROS_INFO_STREAM("getServerQueries printing after processign catalog.");
      rr_catalog_->print();

      return out;
    }

    template <class QueryType>
    void updateQueryResponse(const std::string &server,
                             const QueryType &call)
    {
      //ROS_INFO_STREAM("updateQueryResponse for server " << server);

      std::string serialized_request = temoto_resource_registrar::MessageSerializer::serializeMessage<typename QueryType::Request>(sanityzeData<typename QueryType::Request>(call.request));
      std::string serialized_response = temoto_resource_registrar::MessageSerializer::serializeMessage<typename QueryType::Response>(sanityzeData<typename QueryType::Response>(call.response));
      updateQuery(server, serialized_request, serialized_response);
    }

  protected:
    std::unordered_map<std::string, typename rclcpp::Client<tutorial_interfaces::srv::UnloadComponent>::SharedPtr> unload_clients_;
    std::unordered_map<std::string, typename rclcpp::Client<tutorial_interfaces::srv::StatusComponent>::SharedPtr> status_clients_;
    std::unordered_map<std::string, typename rclcpp::Client<tutorial_interfaces::srv::DataFetchComponent>::SharedPtr> fetch_clients_;

    /**
 * @brief Virtual method that needs to be implemented on every extension of RR_CORE. This method creates and sends the status
 * callback to a target that is defined in the statusData.
 * 
 * @param target_rr 
 * @param request_id 
 * @param statusData 
 * @return true 
 * @return false 
 */
    virtual bool callStatusClient(const std::string &target_rr, const std::string &request_id, Status status_data)
    {

      std::string client_name = IDUtils::generateStatus(target_rr);

      initClient<tutorial_interfaces::srv::StatusComponent>(client_name, status_clients_);

      auto request = std::make_shared<tutorial_interfaces::srv::StatusComponent::Request>();

      //temoto_resource_registrar::StatusComponent status_srv;
      //ROS_INFO_STREAM("callStatusClient");
      auto container = rr_catalog_->findOriginalContainer(status_data.id_);
      if (!container.empty_)
      {
        //ROS_WARN_STREAM("CONTAINER EXISTS");
        request->serialised_request = container.raw_request_;
        request->serialised_response = container.raw_query_;

        status_data.serialised_request_ = container.raw_request_;
        status_data.serialised_response_ = container.raw_query_;
      }
      else
      {
        //ROS_WARN_STREAM("CONTAINER does not EXISTS");
      }

      handleRrServerCb(request_id, status_data);

      request->target = status_data.id_;
      request->status = static_cast<int>(status_data.state_);
      request->message = status_data.message_;

      //ROS_INFO_STREAM("calling status client " << client_name << " target id: " << status_data.id_);
      auto result = status_clients_[client_name]->async_send_request(request);
      return rclcpp::spin_until_future_complete(node_, result) == rclcpp::executor::FutureReturnCode::SUCCESS;
    };

    virtual std::map<std::string, std::pair<std::string, std::string>> callDataFetchClient(const std::string &target_rr,
                                                                                           const std::string &origin_rr,
                                                                                           const std::string &server_name)
    {
      //ROS_INFO_STREAM("callDataFetchClient");

      std::string client_name = IDUtils::generateFetch(target_rr);
      initClient<tutorial_interfaces::srv::DataFetchComponent>(client_name, fetch_clients_);

      auto request = std::make_shared<tutorial_interfaces::srv::DataFetchComponent::Request>();

      //temoto_resource_registrar::DataFetchComponent data_fetch_srv;

      request->origin_rr = origin_rr;
      request->server_name = server_name;

      //ROS_INFO_STREAM("calling data fetch client " << client_name << " - " << data_fetch_srv.request);

      //fetch_clients_[client_name]->call(data_fetch_srv);
      auto result = fetch_clients_[client_name]->async_send_request(request);

      std::map<std::string, std::pair<std::string, std::string>> res;

      if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        int c = 0;
        for (const auto &id : result.get()->ids)
        {
          std::pair<std::string, std::string> req_res(result.get()->serialized_requests.at(c),
                                                      result.get()->serialized_responses.at(c));
          res[id] = req_res;
          c++;
        }
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
      }
    }

  private:
    MultiThreadedExecutor executor;
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Service<tutorial_interfaces::srv::UnloadComponent>::SharedPtr unload_service_;
    rclcpp::Service<tutorial_interfaces::srv::StatusComponent>::SharedPtr status_service_;
    rclcpp::Service<tutorial_interfaces::srv::DataFetchComponent>::SharedPtr fetch_service_;

    template <class ServiceClass>
    void initClient(const std::string &client_name, std::unordered_map<std::string, typename rclcpp::Client<ServiceClass>::SharedPtr> &client_map)
    {
      if (client_map.count(client_name) == 0)
      {
        //ROS_INFO_STREAM("creating client " << client_name);
        typename rclcpp::Client<ServiceClass>::SharedPtr ptr = node_->create_client<ServiceClass>(client_name);

        client_map[client_name] = std::move(ptr);
        //ROS_INFO_STREAM("created client...");
      }
    }

    void startServices()
    {
      node_ = rclcpp::Node::make_shared(name());

      rclcpp::Service<tutorial_interfaces::srv::UnloadComponent>::SharedPtr unload_service_ =
          node_->create_service<tutorial_interfaces::srv::UnloadComponent>(IDUtils::generateUnload(name()), &ResourceRegistrarRos2::unloadCallback);

      rclcpp::Service<tutorial_interfaces::srv::StatusComponent>::SharedPtr status_service_ =
          node_->create_service<tutorial_interfaces::srv::StatusComponent>(IDUtils::generateUnload(name()), &ResourceRegistrarRos2::unloadCallback);

      rclcpp::Service<tutorial_interfaces::srv::DataFetchComponent>::SharedPtr fetch_service_ =
          node_->create_service<tutorial_interfaces::srv::DataFetchComponent>(IDUtils::generateUnload(name()), &ResourceRegistrarRos2::unloadCallback);

      executor.add_node(node_);
    }

    void unloadCallback(const std::shared_ptr<tutorial_interfaces::srv::UnloadComponent::Request> req,
                        std::shared_ptr<tutorial_interfaces::srv::UnloadComponent::Response> res)
    {
      //ROS_INFO_STREAM("printing catalog before unload callback");
      rr_catalog_->print();

      //ROS_INFO_STREAM("unloadCallback " << req.target);
      std::string id = req->target;
      //ROS_INFO_STREAM("std::string id " << id);
      res->status = localUnload(id);
    }

    void statusCallback(const std::shared_ptr<tutorial_interfaces::srv::StatusComponent::Request> req,
                        std::shared_ptr<tutorial_interfaces::srv::StatusComponent::Response> res)
    {
      //ROS_INFO_STREAM("statusCallback: " << req.target);
      handleStatus(req->target, {static_cast<Status::State>(req->status), req->target, req->message, req->serialised_request, req->serialised_response});
    }

    void dataFetchCallback(const std::shared_ptr<tutorial_interfaces::srv::DataFetchComponent::Request> req,
                           std::shared_ptr<tutorial_interfaces::srv::DataFetchComponent::Response> res)
    {
      //ROS_INFO_STREAM("syncCallback " << req);
      std::map<UUID, std::pair<std::string, std::string>> resMap = handleDataFetch(req->origin_rr, req->server_name);
      std::vector<std::string> ids, serialized_requests, serialized_responses;

      for (const auto &el : resMap)
      {
        ids.push_back(el.first);
        serialized_requests.push_back(el.second.first);
        serialized_responses.push_back(el.second.second);
      }

      res->ids = ids;
      res->serialized_requests = serialized_requests;
      res->serialized_responses = serialized_responses;
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