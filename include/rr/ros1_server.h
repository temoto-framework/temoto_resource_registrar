#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_SERVER_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_SERVER_H

#include "ros/ros.h"

#include "temoto_resource_registrar/rr_server_base.h"

#include "rr/message_serializer.h"
#include "rr/ros1_client.h"
#include "rr/ros1_query.h"

#include <sstream>
#include <string>
#include <vector>

/**
 * @brief A wrapper class for the temoto_resource_registrar::RRServerBase. Provides templating to support multiple message types.
 * This class is responsible for executing resource loading and unloading related logic.
 * 
 * @tparam ServiceClass - Type of the resource this server will serve.
 */
template <class ServiceClass>
class Ros1Server : public temoto_resource_registrar::RrServerBase
{
public:
  /**
 * @brief Construct a new Ros 1 Server object. 
 * 
 * @param name - Name of the server being created. These need to be unique on a ResourceRegistrar basis.
 * @param loadCallback - Users load callback being executed when a unique resource is first requested.
 * @param unLoadCallback - User unload callback being executed when the final consumer of a resource is unloaded.
 */
  Ros1Server(const std::string &name,
             void (*loadCallback)(typename ServiceClass::Request &,
                                  typename ServiceClass::Response &),
             void (*unLoadCallback)(typename ServiceClass::Request &,
                                    typename ServiceClass::Response &)) : temoto_resource_registrar::RrServerBase(name, NULL, NULL),
                                                                          typed_load_callback_ptr_(loadCallback),
                                                                          typed_unload_callback_ptr_(unLoadCallback)
  {
    ROS_INFO_STREAM("Starting up server..." << name);
    service_ = nh_.advertiseService(name, &Ros1Server::serverCallback, this);
    ROS_INFO("Starting up server done!!!");
  }

  /**
 * @brief Unloads a specific resource specified by the message id from the catalog.
 * 
 * @param id - ID of the message being unloaded
 * @return true - The unloading unloaded some resources.
 * @return false - No resources were unloaded.
 */
  bool unloadMessage(const std::string &id)
  {
    ROS_INFO_STREAM("Starting to unload id: " << id);

    ROS_INFO_STREAM("checkign for dependencies... ");
    std::string serializedRequest = rr_catalog_->findOriginalContainer(id).rawRequest_;
    std::string serializedResponse = rr_catalog_->unload(name_, id);
    typename ServiceClass::Request request = MessageSerializer::deSerializeMessage<typename ServiceClass::Request>(serializedRequest);
    typename ServiceClass::Response response = MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(serializedResponse);
    ROS_INFO_STREAM("Unload result size: " << serializedResponse.size());

    if (rr_catalog_->canBeUnloaded(name_))
    {

      ROS_INFO_STREAM("Time for unload CB! " << serializedRequest.size());
      ROS_INFO_STREAM("Time for unload CB! " << serializedResponse.size());
      typed_unload_callback_ptr_(request, response);
    }

    return serializedResponse.size() > 0;
  }

  /**
 * @brief Executes the server message handling logic. A request and response are passed to the method. The request is checked
 * for uniqueness int he catalog. If it is unique, a new entry is created and the user load callback is executed. In case
 * it is not unique the corresponding request response is fetched from catalog storage and deserialized for the user.
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
  bool serverCallback(typename ServiceClass::Request &req, typename ServiceClass::Response &res)
  {
    ROS_INFO("In Server Handler!!!");

    std::string serializedRequest = MessageSerializer::serializeMessage<typename ServiceClass::Request>(req);

    std::string generatedId = generateId();
    res.TemotoMetadata.requestId = generatedId;

    Ros1Query<ServiceClass> wrappedQuery = wrap(req, res);

    ROS_INFO_STREAM("checking existance..." << serializedRequest.size());
    std::string requestId = this->rr_catalog_->queryExists(name_, serializedRequest);

    if (requestId.size() == 0)
    {
      ROS_INFO("NOPE, does not exist");

      typed_load_callback_ptr_(req, res);

      rr_catalog_->storeQuery(name_,
                              wrappedQuery,
                              serializedRequest,
                              sanitizeAndSerialize(res));

      ROS_INFO_STREAM("STORED!!!");

      Ros1Query<ServiceClass> wrappedResponse = wrap(req, res);

      if (wrappedResponse.dependencies().size()) {
        ROS_INFO("Dependency storage required:");
        for (auto const &i : wrappedResponse.dependencies())
        {
          ROS_INFO_STREAM("       dependency: " << wrappedResponse.id() << " - " << i.first << " - " << i.second);
          rr_catalog_->storeDependency(wrappedResponse.id(), i.second, i.first);
        }
      }
    }
    else
    {
      ROS_INFO("Request found. No storage needed. Fetching it... ");
      typename ServiceClass::Response fetchedResponse = fetchResponse(requestId, wrappedQuery);
      ROS_INFO("Fetching done...");
      res = fetchedResponse;
    }

    ROS_INFO_STREAM("server call end" << res.TemotoMetadata.requestId);
    return true;
  }

protected:
  void (*typed_load_callback_ptr_)(typename ServiceClass::Request &, typename ServiceClass::Response &);
  void (*typed_unload_callback_ptr_)(typename ServiceClass::Request &, typename ServiceClass::Response &);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;

  void storeQuery(const std::string &rawRequest, Ros1Query<ServiceClass> query) const
  {
    rr_catalog_->storeQuery(name_,
                            query,
                            rawRequest,
                            sanitizeAndSerialize(query.response()));
  }

  Ros1Query<ServiceClass> wrap(typename ServiceClass::Request &req, typename ServiceClass::Response &res)
  {
    ServiceClass srvCall;
    srvCall.request = req;
    srvCall.response = res;

    return Ros1Query<ServiceClass>(srvCall);
  }

      typename ServiceClass::Response fetchResponse(const std::string &requestId, Ros1Query<ServiceClass> query) const
  {
    std::string serializedResponse = rr_catalog_->processExisting(name_, requestId, query);
    typename ServiceClass::Response fetchedResponse = MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(serializedResponse);

    return fetchedResponse;
  }

  static typename ServiceClass::Response sanityzeRequest(typename ServiceClass::Response data)
  {
    typename ServiceClass::Response empty;
    data.TemotoMetadata = empty.TemotoMetadata;
    return data;
  }

  std::string sanitizeAndSerialize(typename ServiceClass::Response res)
  {
    std::string serialized = MessageSerializer::serializeMessage<typename ServiceClass::Response>(res);

    return serialized;
  }
};
#endif