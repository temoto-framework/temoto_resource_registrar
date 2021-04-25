#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_SERVER_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_SERVER_H

#include "ros/ros.h"

#include "temoto_resource_registrar/rr_server_base.h"
#include "temoto_resource_registrar/rr_serializer.h"

#include "rr/message_serializer.h"
#include "rr/query_utils.h"
#include "rr/ros1_client.h"
#include "rr/ros1_query.h"

#include <functional>
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
                                                                          typed_unload_callback_ptr_(unLoadCallback),
                                                                          member_load_cb_(NULL),
                                                                          member_unload_cb_(NULL)
  {
    initialize();
  }

  Ros1Server(const std::string &name,
             std::function<void(typename ServiceClass::Request &,
                                typename ServiceClass::Response &)>
                 member_load_cb,
             std::function<void(typename ServiceClass::Request &,
                                typename ServiceClass::Response &)>
                 member_unload_cb)
      : temoto_resource_registrar::RrServerBase(name, NULL, NULL),
        member_load_cb_(member_load_cb),
        member_unload_cb_(member_unload_cb),
        typed_load_callback_ptr_(NULL),
        typed_unload_callback_ptr_(NULL)
  {
    initialize();
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

    ROS_INFO_STREAM("------------------ Catalog");
    rr_catalog_->print();
    ROS_INFO_STREAM("------------------ Catalog");

    ROS_INFO_STREAM("loading serialized request and response from catalog.. ");

    ROS_INFO_STREAM("unloadMessage");
    temoto_resource_registrar::QueryContainer<std::string> originalContainer = rr_catalog_->findOriginalContainer(id);
    std::string serializedRequest = originalContainer.rawRequest_;
    ROS_INFO_STREAM("Catalog data... isEmpty: " << originalContainer.empty_);
    ROS_INFO_STREAM("Catalog data... q_.id(): " << originalContainer.q_.id());
    ROS_INFO_STREAM("Catalog data... rawQuery_: " << originalContainer.rawQuery_);
    ROS_INFO_STREAM("Catalog data... rawRequest_: " << originalContainer.rawRequest_);

    bool canUnload = false;

    std::string serializedResponse = rr_catalog_->unload(id_, id, canUnload);

    typename ServiceClass::Request request;
    typename ServiceClass::Response response;

    try
    {
      request = MessageSerializer::deSerializeMessage<typename ServiceClass::Request>(serializedRequest);
      response = MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(serializedResponse);

      response.temotoMetadata.requestId = originalContainer.q_.id();

      ROS_INFO_STREAM("Found response with legth: " << serializedResponse.size());
    }
    catch (const temoto_resource_registrar::DeserializationException &e)
    {
      ROS_WARN_STREAM("Request length: " << serializedRequest.size());
      ROS_WARN_STREAM("Response length: " << serializedResponse.size());
      ROS_WARN_STREAM("Some serialization/deserialization issue in server unload");
    }

    if (canUnload)
    {

      ROS_INFO_STREAM("Resource can be unloaded. Executing callback.");
      ROS_INFO_STREAM("req: " << request);
      ROS_INFO_STREAM("res: " << response);

      if (typed_unload_callback_ptr_ != NULL)
      {
        typed_unload_callback_ptr_(request, response);
      }
      else
      {
        member_unload_cb_(request, response);
      }
    }
    else
    {
      ROS_INFO_STREAM("Resource can not be unloaded. Printing catalog for debug");
    }

    rr_catalog_->print();

    ROS_INFO_STREAM("unload done for id: " << id << " result: " << (serializedResponse.size() > 0));
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
    ROS_INFO_STREAM("Starting serverCallback");

    typename ServiceClass::Request sanitizedReq = sanityzeRequest(req);

    bool hasEqualsDefined = QueryUtils::EqualExists<typename ServiceClass::Request>::value;
    ROS_INFO_STREAM("request has equals defined?: " << hasEqualsDefined);

    std::string serializedRequest = MessageSerializer::serializeMessage<typename ServiceClass::Request>(sanitizedReq);
    std::string requestId = "";

    ROS_INFO_STREAM("checking existance of request in catalog... ");
    ROS_INFO_STREAM("Message: " << req);
    ROS_INFO_STREAM("sanitized Message: " << sanitizedReq);

    if (hasEqualsDefined)
    {
      ROS_INFO_STREAM("evaluating uniqueness based on ==");
      std::vector<temoto_resource_registrar::QueryContainer<std::string>> allServerRequests = rr_catalog_->getUniqueServerQueries(id_);
      for (const auto &q : allServerRequests)
      {
        std::string serReq = q.rawRequest_;
        typename ServiceClass::Request request = MessageSerializer::deSerializeMessage<typename ServiceClass::Request>(serReq);
        if (request == sanitizedReq)
        {
          serializedRequest = serReq;
          requestId = q.q_.id();
        }
      }
    }
    else
    {
      ROS_INFO_STREAM("evaluating uniqueness based on string comparison");
      requestId = this->rr_catalog_->queryExists(id_, serializedRequest);
    }

    std::string generatedId = generateId();
    res.temotoMetadata.requestId = generatedId;

    ROS_INFO_STREAM("Generated request id: " << generatedId);

    Ros1Query<ServiceClass> wrappedQuery = wrap(req, res);

    if (requestId.size() == 0)
    {

      try
      {
        ROS_INFO("Executing query startup callback");
        transaction_callback_ptr_(temoto_resource_registrar::TransactionInfo(100, wrappedQuery));

        ROS_INFO("Query is unique. executing callback");
        if (typed_load_callback_ptr_ != NULL)
        {
          typed_load_callback_ptr_(req, res);
        }
        else
        {
          member_load_cb_(req, res);
        }

        ROS_INFO_STREAM("Storing query in catalog...");

        rr_catalog_->storeQuery(id_,
                                wrappedQuery,
                                serializedRequest,
                                sanitizeAndSerialize(res));

        ROS_INFO_STREAM("Stored!");
      }
      catch (const resource_registrar::TemotoErrorStack &e)
      {
        ROS_WARN_STREAM("Server encountered an error while executing a callback: " << e.getMessage());
        wrappedQuery.metadata().errorStack().appendError(e);
        // store metadata object as serialized string in response
        res.temotoMetadata.status = 500;
        res.temotoMetadata.metadata = Serializer::serialize<temoto_resource_registrar::QueryMetadata>(wrappedQuery.metadata());
      }

      ROS_INFO("Executing query finished callback");
      transaction_callback_ptr_(temoto_resource_registrar::TransactionInfo(200, wrappedQuery));
    }
    else
    {
      ROS_INFO("Request found. No storage needed. Fetching it... ");
      typename ServiceClass::Response fetchedResponse = fetchResponse(requestId, wrappedQuery);
      ROS_INFO("Fetching done...");
      fetchedResponse.temotoMetadata.requestId = generatedId;
      res = fetchedResponse;
    }

    ROS_INFO_STREAM("server call end" << res.temotoMetadata.requestId);
    return true;
  }

protected:
  void (*typed_load_callback_ptr_)(typename ServiceClass::Request &, typename ServiceClass::Response &);
  void (*typed_unload_callback_ptr_)(typename ServiceClass::Request &, typename ServiceClass::Response &);

  std::function<void(typename ServiceClass::Request &, typename ServiceClass::Response &)> member_load_cb_;
  std::function<void(typename ServiceClass::Request &, typename ServiceClass::Response &)> member_unload_cb_;

private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;

  virtual void initialize()
  {
    ROS_INFO_STREAM("Starting up server..." << id_);
    service_ = nh_.advertiseService(id_, &Ros1Server::serverCallback, this);
    ROS_INFO_STREAM("Starting up server done!!!");
  }

  Ros1Query<ServiceClass> wrap(typename ServiceClass::Request &req, typename ServiceClass::Response &res)
  {
    return Ros1Query<ServiceClass>(req.temotoMetadata, res.temotoMetadata);
  }

  typename ServiceClass::Response fetchResponse(const std::string &requestId, Ros1Query<ServiceClass> query) const
  {
    std::string serializedResponse = rr_catalog_->processExisting(id_, requestId, query);
    typename ServiceClass::Response fetchedResponse = MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(serializedResponse);

    return fetchedResponse;
  }

  static typename ServiceClass::Request sanityzeRequest(typename ServiceClass::Request data)
  {
    typename ServiceClass::Request empty;
    data.temotoMetadata = empty.temotoMetadata;
    return data;
  }

  std::string sanitizeAndSerialize(typename ServiceClass::Response res)
  {
    typename ServiceClass::Response empty;
    res.temotoMetadata = empty.temotoMetadata;

    std::string serialized = MessageSerializer::serializeMessage<typename ServiceClass::Response>(res);

    return serialized;
  }
};
#endif