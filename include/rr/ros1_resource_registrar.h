#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS1_RESOURCE_REGISTRAR_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS1_RESOURCE_REGISTRAR_H

#include "ros/console.h"
#include "ros/ros.h"

#include "temoto_resource_registrar/rr_base.h"
#include "temoto_resource_registrar/rr_status.h"

#include "temoto_resource_registrar/StatusComponent.h"
#include "temoto_resource_registrar/UnloadComponent.h"

#include "rr/message_serializer.h"
#include "rr/ros1_client.h"
#include "rr/ros1_server.h"

#include <functional>

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
      ROS_INFO_STREAM("Destroying RR Ros " << name());
      ROS_INFO_STREAM("unloading clients");
      for (const std::string &clientId : clients_.getIds())
      {
        try
        {
          ROS_INFO_STREAM("unloading client " << clientId);
          unloadClient(clientId);
        }
        catch (...)
        {
          ROS_WARN_STREAM("unloading error for client " << clientId);
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
 * @param statusFunc - Optional. User defined status function. Executed when a status is sent by a downstream server.
 * @param overrideStatus - Optional. Forces overwriting of the client status function.
 */
    template <class QueryType>
    void call(const std::string &rr,
              const std::string &server,
              QueryType &query,
              std::function<void(QueryType, Status)> statusFunc = NULL,
              bool overrideStatus = false)
    {
      ROS_INFO_STREAM("calling " << rr << " server " << server);
      Ros1Query<QueryType> wrappedBaseQuery(query);
      
      privateCall<Ros1Client<QueryType>,
                  Ros1Server<QueryType>,
                  Ros1Query<QueryType>,
                  std::function<void(QueryType, Status)>>(&rr,
                                                          NULL,
                                                          server,
                                                          wrappedBaseQuery,
                                                          statusFunc,
                                                          overrideStatus);

      query = wrappedBaseQuery.rosQuery();

      ROS_INFO_STREAM("calling " << rr << " server " << server << " done");
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
      ROS_INFO_STREAM("unload Called for rr " << rr << " id: " << id);

      ros::NodeHandle nh;

      std::string clientName = rr + "_unloader";

      if (unload_clients_.count(clientName) == 0)
      {
        ROS_INFO_STREAM("creating unload client...");
        auto sc = nh.serviceClient<UnloadComponent>(clientName);
        auto client = std::make_unique<ros::ServiceClient>(sc);
        unload_clients_[clientName] = std::move(client);
        ROS_INFO_STREAM("created unload client...");
      }

      temoto_resource_registrar::UnloadComponent unloadSrv;
      unloadSrv.request.target = id;

      bool res = unload_clients_[clientName]->call(unloadSrv);
      ROS_INFO_STREAM("result: " << res);

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
      ROS_INFO_STREAM("deSerializeBaseQuery ROS ");
      QueryType q;

      q.request = MessageSerializer::deSerializeMessage<typename QueryType::Request>(container.rawRequest_);
      q.response = MessageSerializer::deSerializeMessage<typename QueryType::Response>(container.rawQuery_);

      q.response.temotoMetadata.requestId = container.q_.id();

      return q;
    }

    template <class QueryType>
    std::vector<QueryType> getServerQueries(const std::string &server)
    {
      ROS_INFO_STREAM("getServerQueries printing before processign catalog.");
      rr_catalog_->print();
      std::vector<QueryType> out;

      for (auto const &queryContainer : rr_catalog_->getUniqueServerQueries(server))
      {
        out.push_back(deSerializeQuery<QueryType>(queryContainer));
      }

      ROS_INFO_STREAM("getServerQueries printing after processign catalog.");
      rr_catalog_->print();

      return out;
    }

    template <class QueryType>
    void updateQueryResponse(const std::string &server,
                             const QueryType &call)
    {
      ROS_INFO_STREAM("updateQueryResponse for server " << server);

      std::string serializedRequest = MessageSerializer::serializeMessage<typename QueryType::Request>(sanityzeData<typename QueryType::Request>(call.request));
      std::string serializedResponse = MessageSerializer::serializeMessage<typename QueryType::Response>(sanityzeData<typename QueryType::Response>(call.response));
      updateQuery(server, serializedRequest, serializedResponse);
    }

  protected:
    std::unordered_map<std::string, std::unique_ptr<ros::ServiceClient>> unload_clients_;
    std::unordered_map<std::string, std::unique_ptr<ros::ServiceClient>> status_clients_;

    /**
 * @brief Virtual method that needs to be implemented on every extension of RR_CORE. This method creates and sends the status
 * callback to a target that is defined in the statusData.
 * 
 * @param clientName 
 * @param statusData 
 * @return true 
 * @return false 
 */
    virtual bool callStatusClient(const std::string &clientName, Status statusData)
    {
      ros::NodeHandle nh;

      if (status_clients_.count(clientName) == 0)
      {
        ROS_INFO_STREAM("creating status client...");
        auto sc = nh.serviceClient<StatusComponent>(clientName);
        auto client = std::make_unique<ros::ServiceClient>(sc);
        status_clients_[clientName] = std::move(client);
        ROS_INFO_STREAM("client " << clientName << " created");
      }
      temoto_resource_registrar::StatusComponent statusSrv;
      ROS_INFO_STREAM("callStatusClient");
      auto container = rr_catalog_->findOriginalContainer(statusData.id_);
      if (!container.empty_)
      {
        ROS_WARN_STREAM("CONTAINER EXISTS");
        statusSrv.request.serialisedRequest = container.rawRequest_;
        statusSrv.request.serialisedResponse = container.rawQuery_;
      }
      else
      {
        ROS_WARN_STREAM("CONTAINER does not EXISTS");
      }

      statusSrv.request.target = statusData.id_;
      statusSrv.request.status = static_cast<int>(statusData.state_);
      statusSrv.request.message = statusData.message_;

      ROS_INFO_STREAM("calling status client " << clientName << " target id: " << statusData.id_);

      return status_clients_[clientName]->call(statusSrv);
    };

    /**
 * @brief ROS 1 implementation of the unloadResource method. 
 * 
 * @param id 
 * @param dependency 
 */
    virtual void unloadResource(const std::string &id, const std::pair<const std::string, std::string> &dependency)
    {
      bool unloadStatus = unload(dependency.second, dependency.first);
      if (unloadStatus)
      {
        rr_catalog_->unloadDependency(id, dependency.first);
      }
    }

  private:
    ros::ServiceServer unload_service_;
    ros::ServiceServer status_service_;

    void startServices()
    {
      ROS_INFO_STREAM("Starting up services...");
      ros::NodeHandle nh;
      ROS_INFO_STREAM("Starting unload_service_... " << name() + "_unloader");
      unload_service_ = nh.advertiseService(name() + "_unloader", &ResourceRegistrarRos1::unloadCallback, this);
      ROS_INFO_STREAM("Starting status_service_... " << name() + "_status");
      status_service_ = nh.advertiseService(name() + "_status", &ResourceRegistrarRos1::statusCallback, this);
    }

    bool unloadCallback(UnloadComponent::Request &req, UnloadComponent::Response &res)
    {
      ROS_INFO_STREAM("printing catalog before unload callback");
      rr_catalog_->print();

      ROS_INFO_STREAM("unloadCallback " << req.target);
      std::string id = req.target;
      ROS_INFO_STREAM("std::string id " << id);
      res.status = localUnload(id);
      return true;
    }

    bool statusCallback(StatusComponent::Request &req, StatusComponent::Response &res)
    {
      ROS_INFO_STREAM("statusCallback " << req.target);
      handleStatus({static_cast<Status::State>(req.status), req.target, req.message, req.serialisedRequest, req.serialisedResponse});
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
      data.temotoMetadata = empty.temotoMetadata;
      return data;
    }
  };
}
#endif