#include "ros/console.h"
#include "ros/ros.h"

#include "temoto_resource_registrar/rr_base.h"

#include "temoto_resource_registrar/UnloadComponent.h"

#include "rr/ros1_client.h"
#include "rr/ros1_server.h"

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
      startUnloadService();
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
              StatusFunction statusFunc = NULL,
              bool overrideStatus = false)
    {

      query.response.TemotoMetadata.servingRr = name();

      Ros1Query<QueryType> wrappedBaseQuery(query.request, query.response);

      ROS_INFO_STREAM("-----call RES" << query.response.TemotoMetadata.requestId << " ; " << query.response.TemotoMetadata.servingRr << " ; " << query.response.TemotoMetadata.originRr);
      ROS_INFO_STREAM("-----call REQ" << query.request.TemotoMetadata.requestId << " ; " << query.request.TemotoMetadata.servingRr << " ; " << query.request.TemotoMetadata.originRr);

      privateCall<Ros1Client<QueryType>,
                  Ros1Server<QueryType>,
                  Ros1Query<QueryType>>(&rr,
                                        NULL,
                                        server,
                                        wrappedBaseQuery,
                                        parentQuery,
                                        statusFunc,
                                        overrideStatus);

      ROS_INFO_STREAM("-----call RES" << query.response.TemotoMetadata.requestId << " ; " << query.response.TemotoMetadata.servingRr << " ; " << query.response.TemotoMetadata.originRr);
      ROS_INFO_STREAM("-----call REQ" << query.request.TemotoMetadata.requestId << " ; " << query.request.TemotoMetadata.servingRr << " ; " << query.request.TemotoMetadata.originRr);

      if (parentQuery != NULL)
      {
        ROS_INFO_STREAM("DEPENDENCY SIZE?  " << parentQuery->dependencies().size());
        for (const auto &el : parentQuery->dependencies())
          ROS_INFO_STREAM("" << el.first << "::" << el.second);
      }

      QueryType sc;
      sc.request = wrappedBaseQuery.request();
      sc.response = wrappedBaseQuery.response();
      sc.response.TemotoMetadata.dependencies = convertDependencies(parentQuery);
      query = sc;
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
      ROS_INFO_STREAM("unload Called");

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

      ROS_INFO_STREAM("executing unload call");

      bool res = unload_clients_[clientName]->call(unloadSrv);

      ROS_INFO_STREAM("executing unload call done " << res);

      return res;
    }

  protected:
    std::unordered_map<std::string, std::unique_ptr<ros::ServiceClient>> unload_clients_;

  private:
    ros::ServiceServer service_;

    void startUnloadService()
    {
      ros::NodeHandle nh;
      service_ = nh.advertiseService(name() + "_unloader", &ResourceRegistrarRos1::unloadCallback, this);
    }

    bool unloadCallback(UnloadComponent::Request &req, UnloadComponent::Response &res)
    {
      std::string id = req.target;
      res.status = localUnload(id);
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