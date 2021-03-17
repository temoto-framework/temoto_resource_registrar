#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS1_RESOURCE_REGISTRAR_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS1_RESOURCE_REGISTRAR_H

#include "ros/console.h"
#include "ros/ros.h"

#include "temoto_resource_registrar/rr_base.h"
#include "temoto_resource_registrar/rr_status.h"

#include "temoto_resource_registrar/StatusComponent.h"
#include "temoto_resource_registrar/UnloadComponent.h"

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
 * - Check if a parent query was bound to the main query and store the query information. This is to detect dependencies in query chains.
 * - Check if a status function was attached to the query. This status function will be called if a status update is sent upstream from the server.
 * 
 * Status functions can be defined for every client. Every client can have only one status function. If it is needed to re-define said function
 * it can be done by raising the overrideStatus parameter. This change applies to all requests done with that client.
 * 
 * @tparam QueryType - Type of the query being executed. It is a normal ROS1 srv.
 * @param rr - Name of the target ResourceRegistrar.
 * @param server - Name of the target Ros1Server.
 * @param query - User query as a QueryType srv object.
 * @param parentQuery - Optional. Used for dependency resolving.
 * @param statusFunc - Optional. User defined status function. Executed when a status is sent by a downstream server.
 * @param overrideStatus - Optional. Forces overwriting of the client status function.
 */
    template <class QueryType>
    void call(const std::string &rr,
              const std::string &server,
              QueryType &query,
              RrQueryBase *parentQuery = NULL,
              std::function<void(QueryType, StatusTodo)> statusFunc = NULL,
              bool overrideStatus = false)
    {
      Ros1Query<QueryType> wrappedBaseQuery(query);


      privateCall<Ros1Client<QueryType>,
                  Ros1Server<QueryType>,
                  Ros1Query<QueryType>,
                  std::function<void(QueryType, StatusTodo)>>(&rr,
                                                              NULL,
                                                              server,
                                                              wrappedBaseQuery,
                                                              parentQuery,
                                                              statusFunc,
                                                              overrideStatus);

      query = wrappedBaseQuery.rosQuery();
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
      ROS_INFO_STREAM("unload Called ");

      ros::NodeHandle nh;

      std::string clientName = rr + "_unloader";

      if (unload_clients_.count(clientName) == 0)
      {
        ROS_INFO_STREAM("creating unload client...");
        auto sc = nh.serviceClient<UnloadComponent>(clientName);
        auto client = std::make_unique<ros::ServiceClient>(sc);
        unload_clients_[clientName] = std::move(client);
      }

      temoto_resource_registrar::UnloadComponent unloadSrv;
      unloadSrv.request.target = id;

      bool res = unload_clients_[clientName]->call(unloadSrv);

      return res;
    }

    virtual void sendStatus(const std::string &id, Status status, std::string &message)
    {

      ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!!! sendStatus " << id << " STARTT");
      std::unordered_map<std::string, std::string> notifyIds = rr_catalog_->getAllQueryIds(id);

      ros::NodeHandle nh;

      ROS_INFO_STREAM("Notify ids---");
      for (auto const &notId : notifyIds)
      {
        std::string originalId = rr_catalog_->getOriginQueryId(notId.first);
        ROS_INFO_STREAM(notId.first << ": Original: " << originalId << ": serverId: " << notId.second);

        std::string clientName = notId.second + "_status";

        if (unload_clients_.count(clientName) == 0)
        {
          ROS_INFO_STREAM("creating status client...");
          auto sc = nh.serviceClient<StatusComponent>(clientName);
          auto client = std::make_unique<ros::ServiceClient>(sc);
          unload_clients_[clientName] = std::move(client);
        }
        temoto_resource_registrar::StatusComponent statusSrv;

        statusSrv.request.target = notId.first;
        statusSrv.request.status = static_cast<int>(status);
        statusSrv.request.message = message;

        unload_clients_[clientName]->call(statusSrv);
      }

      ROS_INFO_STREAM("Notify ids---");

      ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!!! sendStatus " << id << " END");
    }

    //virtual void handleStatus(StatusTodo status)
    virtual void handleStatus(const std::string &id, Status status, std::string &message)
    {

      ROS_INFO_STREAM("<<<<<<<<<<<<<<<<<<<handleStatusSTART>>>>>>>>>>>>>>>>>>>");

      ROS_INFO_STREAM("GOT FOLLOWING MESSAGE: " << id << " - " << static_cast<int>(status) << " - " << message);
      std::string originalId = rr_catalog_->getOriginQueryId(id);
      auto originalQueryWrapper = rr_catalog_->findOriginalContainer(id);
      if (originalId.size())
      {
        std::unordered_map<std::string, std::string> dependencyMap = rr_catalog_->getDependencies(originalId);
        std::string firstId = dependencyMap.begin()->first;
        // stop attempt if dependency
        if (id != firstId)
        {
          return;
        }
      }

      std::string clientName = rr_catalog_->getIdClient(id);

      if (clients_.exists(clientName))
      {
        ROS_INFO_STREAM("executing user callback, if it exists " << clients_.hasCallback(clientName));

        temoto_resource_registrar::StatusTodo statusInfo = {temoto_resource_registrar::StatusTodo::State::OK, id, message};

        clients_.runCallback(clientName, statusInfo);
      }
      else
      {
        ROS_ERROR_STREAM("THIS IS BAD; NO CLIENT FOR QUERY");
      }

      if (originalId.size())
      {
        sendStatus(originalId, status, message);
      }

      ROS_INFO_STREAM("<<<<<<<<<<<<<<<<<<<handleStatusEND>>>>>>>>>>>>>>>>>>>");
    }

  protected:
    std::unordered_map<std::string, std::unique_ptr<ros::ServiceClient>> unload_clients_;
    std::unordered_map<std::string, std::unique_ptr<ros::ServiceClient>> status_clients_;

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
      ros::NodeHandle nh;
      unload_service_ = nh.advertiseService(name() + "_unloader", &ResourceRegistrarRos1::unloadCallback, this);
      status_service_ = nh.advertiseService(name() + "_status", &ResourceRegistrarRos1::statusCallback, this);
    }

    bool unloadCallback(UnloadComponent::Request &req, UnloadComponent::Response &res)
    {
      ROS_INFO_STREAM("unloadCallback " << req.target);
      std::string id = req.target;
      ROS_INFO_STREAM("std::string id " << id);
      res.status = localUnload(id);
      return true;
    }

    bool statusCallback(StatusComponent::Request &req, StatusComponent::Response &res)
    {
      ROS_INFO_STREAM("statusCallback " << req.target);
      std::string message = "";
      handleStatus(req.target, static_cast<Status>(req.status), req.message);
      return true;
    }

    std::vector<std::string> convertDependencies(RrQueryBase *query)
    {
      std::vector<std::string> dependencies;
      if (query != NULL)
      {
        ROS_INFO_STREAM("query exists with size: " << query->dependencies().size());

        for (auto const mapEntry : query->dependencies())
        {
          dependencies.push_back(mapEntry.first + ";;" + mapEntry.second);
        }
      }
      return dependencies;
    }
  };
}
#endif