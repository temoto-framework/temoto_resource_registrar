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

namespace temoto_resource_registrar
{

  /**
   * @brief Implementation of the temoto_resource_registrar::RrBase to work on ROS1. 
   * The init() function should be called once ROS is initialized. Else unload and other
   * internal services will not be started up.
   * 
   */
  class ResourceRegistrarRos2 : public RrBase, public rclcpp::Node
  {
  public:
    /**
   * @brief Construct a new Resource Registrar Ros 2 object. Take into account that names need to be unique. 
   * They are used as adress fragments for inter-register communications.
   * 
   * @param name 
   */
    ResourceRegistrarRos2(const std::string &name) : RrBase(name), Node(name)
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
 */
    template <class QueryType>
    Ros2Query<QueryType> call(const std::string &rr,
                              const std::string &server,
                              std::shared_ptr<typename QueryType::Request> query,
                              std::function<void(const std::shared_ptr<typename QueryType::Request> &,
                                                 std::shared_ptr<typename QueryType::Response>,
                                                 const temoto_resource_registrar::Status &)>
                                  status_func = NULL)
    {
      //ROS_INFO_STREAM("calling " << rr << " server " << server);

      Ros2Query<QueryType> wrapped_base_query(query);

      privateCall<Ros2Client<QueryType>,
                  Ros2Server<QueryType>,
                  Ros2Query<QueryType>,
                  std::function<void(const std::shared_ptr<typename QueryType::Request> &,
                                     std::shared_ptr<typename QueryType::Response>,
                                     const temoto_resource_registrar::Status &)>>(&rr,
                                                                                  NULL,
                                                                                  server,
                                                                                  wrapped_base_query,
                                                                                  status_func);

      return wrapped_base_query;

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

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "unload called");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), rr);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), id);

      std::string client_name = IDUtils::generateUnload(rr);
      initClient<tutorial_interfaces::srv::UnloadComponent>(client_name, unload_clients_, unload_callback_group_);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "client %s created", client_name.c_str());

      auto request = std::make_shared<tutorial_interfaces::srv::UnloadComponent::Request>();
      request->target = id;

      bool res = false;
      bool query_complete = false;

      auto inner_client_callback = [&, this](rclcpp::Client<tutorial_interfaces::srv::UnloadComponent>::SharedFuture inner_future)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[inner service] callback executed");
        auto result = inner_future.get();
        res = result->status;
        query_complete = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[inner service] callback finished");
      };

      auto result = unload_clients_[client_name]->async_send_request(request, inner_client_callback);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "waiting for future");

      while (!query_complete && rclcpp::ok())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      return res;

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
    std::pair<
        std::shared_ptr<typename QueryType::Request>,
        std::shared_ptr<typename QueryType::Response>>
    deSerializeQuery(const QueryContainer<RawData> &container)
    {
      //ROS_INFO_STREAM("deSerializeBaseQuery ROS ");
      std::pair<
          std::shared_ptr<typename QueryType::Request>,
          std::shared_ptr<typename QueryType::Response>>
          queryPair(temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename QueryType::Request>(container.raw_request_),
                    temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename QueryType::Response>(container.raw_query_));

      queryPair.second->temoto_metadata.request_id = container.q_.id();

      return queryPair;
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
      std::map<std::string,
               std::pair<
                   std::shared_ptr<typename QueryType::Request>,
                   std::shared_ptr<typename QueryType::Response>>>
          ros_queries;

      //ROS_DEBUG_STREAM("Deserializing strings to query type");
      for (const auto &el : serialised_queries)
      {
        std::pair<
            std::shared_ptr<typename QueryType::Request>,
            std::shared_ptr<typename QueryType::Response>>
            request_response_pair(
                temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename QueryType::Request>(el.second.first),
                temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename QueryType::Response>(el.second.second));

        ros_queries[el.first] = request_response_pair;
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
      std::vector<std::pair<
          std::shared_ptr<typename QueryType::Request>,
          std::shared_ptr<typename QueryType::Response>>>
          out;

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
                             const std::pair<
                                 std::shared_ptr<typename QueryType::Request>,
                                 std::shared_ptr<typename QueryType::Response>> &call)
    {
      //ROS_INFO_STREAM("updateQueryResponse for server " << server);

      std::string serialized_request = temoto_resource_registrar::MessageSerializer::serializeMessage<typename QueryType::Request>(sanityzeData<typename QueryType::Request>(call.first));
      std::string serialized_response = temoto_resource_registrar::MessageSerializer::serializeMessage<typename QueryType::Response>(sanityzeData<typename QueryType::Response>(call.second));
      updateQuery(server, serialized_request, serialized_response);
    }

  protected:
    std::unordered_map<std::string, typename rclcpp::Client<tutorial_interfaces::srv::UnloadComponent>::SharedPtr> unload_clients_;
    std::unordered_map<std::string, typename rclcpp::Client<tutorial_interfaces::srv::StatusComponent>::SharedPtr> status_clients_;
    std::unordered_map<std::string, typename rclcpp::Client<tutorial_interfaces::srv::DataFetchComponent>::SharedPtr> fetch_clients_;

    rclcpp::callback_group::CallbackGroup::SharedPtr unload_callback_group_;
    rclcpp::callback_group::CallbackGroup::SharedPtr status_callback_group_;
    rclcpp::callback_group::CallbackGroup::SharedPtr fetch_callback_group_;

    std::string string_to_hex(const std::string &input)
    {
      static const char hex_digits[] = "0123456789ABCDEF";

      std::string output;
      output.reserve(input.length() * 2);
      for (unsigned char c : input)
      {
        output.push_back(hex_digits[c >> 4]);
        output.push_back(hex_digits[c & 15]);
      }
      return output;
    }

#include <stdexcept>

    int hex_value(unsigned char hex_digit)
    {
      static const signed char hex_values[256] = {
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          0,
          1,
          2,
          3,
          4,
          5,
          6,
          7,
          8,
          9,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          10,
          11,
          12,
          13,
          14,
          15,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          10,
          11,
          12,
          13,
          14,
          15,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
          -1,
      };
      int value = hex_values[hex_digit];
      if (value == -1)
        throw std::invalid_argument("invalid hex digit");
      return value;
    }

    std::string hex_to_string(const std::string &input)
    {
      const auto len = input.length();
      if (len & 1)
        throw std::invalid_argument("odd length");

      std::string output;
      output.reserve(len / 2);
      for (auto it = input.begin(); it != input.end();)
      {
        int hi = hex_value(*it++);
        int lo = hex_value(*it++);
        output.push_back(hi << 4 | lo);
      }
      return output;
    }

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

      initClient<tutorial_interfaces::srv::StatusComponent>(client_name, status_clients_, status_callback_group_);

      auto request = std::make_shared<tutorial_interfaces::srv::StatusComponent::Request>();

      //temoto_resource_registrar::StatusComponent status_srv;
      //ROS_INFO_STREAM("callStatusClient");
      auto container = rr_catalog_->findOriginalContainer(status_data.id_);
      if (!container.empty_)
      {
        //ROS_WARN_STREAM("CONTAINER EXISTS");
        request->serialised_request = string_to_hex(container.raw_request_);
        request->serialised_response = string_to_hex(container.raw_query_);

        status_data.serialised_request_ = container.raw_request_;
        status_data.serialised_response_ = container.raw_query_;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "container is there. Here are sizes:");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%i", request->serialised_request.size());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%i", request->serialised_response.size());
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
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling client with id: " + client_name + " target id: " + status_data.id_);
      auto result = status_clients_[client_name]->async_send_request(request);
      return true;
      //return rclcpp::spin_until_future_complete(this, result) == rclcpp::executor::FutureReturnCode::SUCCESS;
    };

    virtual std::map<std::string, std::pair<std::string, std::string>> callDataFetchClient(const std::string &target_rr,
                                                                                           const std::string &origin_rr,
                                                                                           const std::string &server_name)
    {
      //ROS_INFO_STREAM("callDataFetchClient");

      std::string client_name = IDUtils::generateFetch(target_rr);
      initClient<tutorial_interfaces::srv::DataFetchComponent>(client_name, fetch_clients_, fetch_callback_group_);

      auto request = std::make_shared<tutorial_interfaces::srv::DataFetchComponent::Request>();

      //temoto_resource_registrar::DataFetchComponent data_fetch_srv;

      request->origin_rr = origin_rr;
      request->server_name = server_name;

      //ROS_INFO_STREAM("calling data fetch client " << client_name << " - " << data_fetch_srv.request);

      //fetch_clients_[client_name]->call(data_fetch_srv);
      auto result = fetch_clients_[client_name]->async_send_request(request);

      std::map<std::string, std::pair<std::string, std::string>> res;
      /*
      if (rclcpp::spin_until_future_complete(this, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
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
*/
      return res;
    }

    /**
 * @brief ROS 2 implementation of the unloadResource method. 
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
    rclcpp::Service<tutorial_interfaces::srv::UnloadComponent>::SharedPtr unload_service_;
    rclcpp::Service<tutorial_interfaces::srv::StatusComponent>::SharedPtr status_service_;
    rclcpp::Service<tutorial_interfaces::srv::DataFetchComponent>::SharedPtr fetch_service_;

    std::shared_ptr<ResourceRegistrarRos2> shared_ptr_;

    //rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_;
    //std::shared_ptr<rclcpp::Node> node_;
    //rclcpp::executors::MultiThreadedExecutor exec_;

    template <class ServiceClass>
    void initClient(
        const std::string &client_name,
        std::unordered_map<std::string, typename rclcpp::Client<ServiceClass>::SharedPtr> &client_map,
        rclcpp::callback_group::CallbackGroup::SharedPtr group)
    {
      if (client_map.count(client_name) == 0)
      {
        //ROS_INFO_STREAM("creating client " << client_name);
        typename rclcpp::Client<ServiceClass>::SharedPtr ptr = this->create_client<ServiceClass>(client_name, rmw_qos_profile_services_default, group);

        client_map[client_name] = std::move(ptr);
        //ROS_INFO_STREAM("created client...");
      }
    }

    void startServices()
    {

      //node_ = rclcpp::Node::make_shared(name() + "_internal");

      //callback_group_ = node_->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
      unload_callback_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
      status_callback_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
      fetch_callback_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting services....");

      unload_service_ =
          this->create_service<tutorial_interfaces::srv::UnloadComponent>(IDUtils::generateUnload(name()),
                                                                          std::bind(&ResourceRegistrarRos2::unloadCallback, this, std::placeholders::_1, std::placeholders::_2)/*,
                                                                          rmw_qos_profile_services_default,
                                                                          unload_callback_group_*/);

      status_service_ =
          this->create_service<tutorial_interfaces::srv::StatusComponent>(IDUtils::generateStatus(name()),
                                                                          std::bind(&ResourceRegistrarRos2::statusCallback, this, std::placeholders::_1, std::placeholders::_2)/*,
                                                                          rmw_qos_profile_services_default,
                                                                          status_callback_group_*/);

      fetch_service_ =
          this->create_service<tutorial_interfaces::srv::DataFetchComponent>(IDUtils::generateFetch(name()),
                                                                             std::bind(&ResourceRegistrarRos2::dataFetchCallback, this, std::placeholders::_1, std::placeholders::_2)/*,
                                                                             rmw_qos_profile_services_default,
                                                                             fetch_callback_group_*/);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "completed....");

      //exec_.add_node(node_);
      //exec_.spin();
    }

    void unloadCallback(const std::shared_ptr<tutorial_interfaces::srv::UnloadComponent::Request> req,
                        std::shared_ptr<tutorial_interfaces::srv::UnloadComponent::Response> res)
    {
      //ROS_INFO_STREAM("printing catalog before unload callback");

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "unload called");
      rr_catalog_->print();

      //ROS_INFO_STREAM("unloadCallback " << req.target);
      std::string id = req->target;
      //ROS_INFO_STREAM("std::string id " << id);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "local unload for id %s", id.c_str());
      res->status = localUnload(id);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "finished unload");
    }

    void statusCallback(const std::shared_ptr<tutorial_interfaces::srv::StatusComponent::Request> req,
                        std::shared_ptr<tutorial_interfaces::srv::StatusComponent::Response> res)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "got status");
      //ROS_INFO_STREAM("statusCallback: " << req.target);
      int status = req->status;
      std::string target = req->target;
      std::string message = req->message;
      std::string serialised_request = hex_to_string(req->serialised_request);
      std::string serialised_response = hex_to_string(req->serialised_response);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t" + target);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t" + message);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t" + serialised_request);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t" + serialised_response);

      handleStatus(target, {static_cast<Status::State>(status), target, message, serialised_request, serialised_response});
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
    static MsgClass sanityzeData(const std::shared_ptr<MsgClass> &data)
    {
      MsgClass empty;

      std::shared_ptr<MsgClass> copy = std::make_shared<MsgClass>(*data);

      copy->temoto_metadata = empty.temoto_metadata;
      return copy;
    }
  };
}
#endif