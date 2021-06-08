#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_SERVER_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_SERVER_H

#include "ros/ros.h"

#include "temoto_resource_registrar/rr_serializer.h"
#include "temoto_resource_registrar/rr_server_base.h"

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
             void (*load_callback)(typename ServiceClass::Request &,
                                   typename ServiceClass::Response &),
             void (*unLoad_callback)(typename ServiceClass::Request &,
                                     typename ServiceClass::Response &)) : temoto_resource_registrar::RrServerBase(name, NULL, NULL),
                                                                           typed_load_callback_ptr_(load_callback),
                                                                           typed_unload_callback_ptr_(unLoad_callback),
                                                                           member_load_cb_(NULL),
                                                                           member_unload_cb_(NULL),
                                                                           member_status_cb_(NULL)
  {
    initialize();
  }

  Ros1Server(const std::string &name,
             std::function<void(typename ServiceClass::Request &,
                                typename ServiceClass::Response &)>
                 member_load_cb,
             std::function<void(typename ServiceClass::Request &,
                                typename ServiceClass::Response &)>
                 member_unload_cb,
             std::function<void(typename ServiceClass::Request &,
                                typename ServiceClass::Response &,
                                const temoto_resource_registrar::Status &)>
                 member_status_cb = NULL)
      : temoto_resource_registrar::RrServerBase(name, NULL, NULL),
        member_load_cb_(member_load_cb),
        member_unload_cb_(member_unload_cb),
        typed_load_callback_ptr_(NULL),
        typed_unload_callback_ptr_(NULL),
        member_status_cb_(member_status_cb)
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
    TEMOTO_DEBUG_STREAM_("Starting to unload id: " << id);

    TEMOTO_DEBUG_STREAM_("------------------ Catalog");
    rr_catalog_->print();
    TEMOTO_DEBUG_STREAM_("------------------ Catalog");

    TEMOTO_DEBUG_STREAM_("loading serialized request and response from catalog.. ");

    TEMOTO_DEBUG_STREAM_("unloadMessage");
    temoto_resource_registrar::QueryContainer<std::string> original_container = rr_catalog_->findOriginalContainer(id);
    std::string serialized_request = original_container.raw_request_;
    TEMOTO_DEBUG_STREAM_("Catalog data... isEmpty: " << original_container.empty_);
    TEMOTO_DEBUG_STREAM_("Catalog data... q_.id(): " << original_container.q_.id());
    TEMOTO_DEBUG_STREAM_("Catalog data... rawQuery_: " << original_container.raw_query_);
    TEMOTO_DEBUG_STREAM_("Catalog data... rawRequest_: " << original_container.raw_request_);

    bool can_unload = false;

    std::string serialized_response = rr_catalog_->unload(id_, id, can_unload);

    typename ServiceClass::Request request;
    typename ServiceClass::Response response;

    try
    {
      request = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Request>(serialized_request);
      response = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(serialized_response);

      response.temoto_metadata.request_id = original_container.q_.id();

      TEMOTO_DEBUG_STREAM_("Found response with legth: " << serialized_response.size());
    }
    catch (const temoto_resource_registrar::DeserializationException &e)
    {
      TEMOTO_DEBUG_STREAM_("Request length: " << serialized_request.size());
      TEMOTO_DEBUG_STREAM_("Response length: " << serialized_response.size());
      TEMOTO_DEBUG_STREAM_("Some serialization/deserialization issue in server unload");
    }

    if (can_unload)
    {

      TEMOTO_DEBUG_STREAM_("Resource can be unloaded. Executing callback.");
      TEMOTO_DEBUG_STREAM_("req: " << request);
      TEMOTO_DEBUG_STREAM_("res: " << response);

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
      TEMOTO_DEBUG_STREAM_("Resource can not be unloaded. Printing catalog for debug");
    }

    rr_catalog_->print();
    rr_catalog_->saveCatalog();

    TEMOTO_DEBUG_STREAM_("unload done for id: " << id << " result: " << (serialized_response.size() > 0));
    return serialized_response.size() > 0;
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
    TEMOTO_DEBUG_STREAM_("Starting serverCallback " << id());

    TEMOTO_DEBUG_STREAM_("generating ID");
    std::string generated_id = generateId();
    TEMOTO_DEBUG_STREAM_("attaching ID to query");
    res.temoto_metadata.request_id = generated_id;

    TEMOTO_DEBUG_STREAM_("wrapping query into a general form");
    Ros1Query<ServiceClass> wrapped_query = wrap(req, res);
    TEMOTO_DEBUG_STREAM_("Wrap complete, can start processing");

#ifdef temoto_enable_tracing
    TEMOTO_LOG_ATTR.startTracingSpan(GET_NAME_FF, wrapped_query.requestMetadata().getSpanContext());
#endif

    TEMOTO_DEBUG_STREAM_("sanitizing request");
    typename ServiceClass::Request sanitized_req = sanityzeRequest(req);

    bool has_equals_defined = QueryUtils::EqualExists<typename ServiceClass::Request>::value;
    TEMOTO_DEBUG_STREAM_("request has equals defined?: " << has_equals_defined);

    std::string serialized_request = temoto_resource_registrar::MessageSerializer::serializeMessage<typename ServiceClass::Request>(sanitized_req);
    std::string request_id = "";

    TEMOTO_DEBUG_STREAM_("checking existance of request in catalog... ");
    TEMOTO_DEBUG_STREAM_("Message: " << req);
    TEMOTO_DEBUG_STREAM_("sanitized Message: " << sanitized_req);

    if (has_equals_defined)
    {
      TEMOTO_DEBUG_STREAM_("evaluating uniqueness based on ==");
      std::vector<temoto_resource_registrar::QueryContainer<std::string>> all_server_requests = rr_catalog_->getUniqueServerQueries(id_);
      for (const auto &q : all_server_requests)
      {
        std::string ser_req = q.raw_request_;
        typename ServiceClass::Request request = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Request>(ser_req);
        if (request == sanitized_req)
        {
          serialized_request = ser_req;
          request_id = q.q_.id();
        }
      }
    }
    else
    {
      TEMOTO_DEBUG_STREAM_("evaluating uniqueness based on string comparison");
      request_id = this->rr_catalog_->queryExists(id_, serialized_request);
    }

    TEMOTO_DEBUG_STREAM_("Generated request id: " << generated_id);

    if (request_id.size() == 0)
    {
      try
      {
        TEMOTO_DEBUG_("Executing query startup callback");
        transaction_callback_ptr_(temoto_resource_registrar::TransactionInfo(100, wrapped_query));

        TEMOTO_DEBUG_("Query is unique. executing callback");
        if (typed_load_callback_ptr_ != NULL)
        {
          typed_load_callback_ptr_(req, res);
        }
        else
        {
          member_load_cb_(req, res);
        }

        TEMOTO_DEBUG_STREAM_("Storing query in catalog...");

        rr_catalog_->storeQuery(id_,
                                wrapped_query,
                                serialized_request,
                                sanitizeAndSerialize(res));

        TEMOTO_DEBUG_STREAM_("Stored!");
      }
      catch (const resource_registrar::TemotoErrorStack &e)
      {
        TEMOTO_DEBUG_STREAM_("Server encountered an error while executing a callback: " << e.getMessage());
        wrapped_query.responseMetadata().errorStack().appendError(e);
        // store metadata object as serialized string in response
        res.temoto_metadata.status = 500;
        res.temoto_metadata.metadata = Serializer::serialize<temoto_resource_registrar::ResponseMetadata>(wrapped_query.responseMetadata());
      }

      TEMOTO_DEBUG_("Executing query finished callback");
      transaction_callback_ptr_(temoto_resource_registrar::TransactionInfo(200, wrapped_query));
    }
    else
    {
      TEMOTO_DEBUG_("Request found. No storage needed. Fetching it... ");
      typename ServiceClass::Response fetched_response = fetchResponse(request_id, wrapped_query);
      TEMOTO_DEBUG_("Fetching done...");
      fetched_response.temoto_metadata.request_id = generated_id;
      res = fetched_response;
    }

    rr_catalog_->saveCatalog();
    TEMOTO_DEBUG_STREAM_("server call end " << res.temoto_metadata.request_id << " " << id());

#ifdef temoto_enable_tracing
    TEMOTO_LOG_ATTR.popParentSpan();
#endif

    return true;
  }

  void triggerCallback(const temoto_resource_registrar::Status &status) const
  {
    TEMOTO_DEBUG_STREAM_("Triggering callback logic..." << id());
    if (member_status_cb_ != NULL)
    {
      typename ServiceClass::Request request = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Request>(status.serialised_request_);
      typename ServiceClass::Response response = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(status.serialised_response_);
      member_status_cb_(request, response, status);
    }
  }

protected:
  void (*typed_load_callback_ptr_)(typename ServiceClass::Request &, typename ServiceClass::Response &);
  void (*typed_unload_callback_ptr_)(typename ServiceClass::Request &, typename ServiceClass::Response &);

  std::function<void(typename ServiceClass::Request &, typename ServiceClass::Response &)> member_load_cb_;
  std::function<void(typename ServiceClass::Request &, typename ServiceClass::Response &)> member_unload_cb_;
  std::function<void(typename ServiceClass::Request &, typename ServiceClass::Response &, const temoto_resource_registrar::Status &)> member_status_cb_;

private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;

  virtual void initialize()
  {
    TEMOTO_DEBUG_STREAM_("Starting up server..." << id_);
    service_ = nh_.advertiseService(id_, &Ros1Server::serverCallback, this);
    TEMOTO_DEBUG_STREAM_("Starting up server done!!!");
  }

  Ros1Query<ServiceClass> wrap(typename ServiceClass::Request &req, typename ServiceClass::Response &res)
  {
    return Ros1Query<ServiceClass>(req.temoto_metadata, res.temoto_metadata);
  }

  typename ServiceClass::Response fetchResponse(const std::string &requestId, Ros1Query<ServiceClass> query) const
  {
    std::string serialized_response = rr_catalog_->processExisting(id_, requestId, query);
    typename ServiceClass::Response fetched_response = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(serialized_response);

    return fetched_response;
  }

  static typename ServiceClass::Request sanityzeRequest(typename ServiceClass::Request data)
  {
    typename ServiceClass::Request empty;
    data.temoto_metadata = empty.temoto_metadata;
    return data;
  }

  std::string sanitizeAndSerialize(typename ServiceClass::Response res)
  {
    typename ServiceClass::Response empty;
    res.temoto_metadata = empty.temoto_metadata;

    std::string serialized = temoto_resource_registrar::MessageSerializer::serializeMessage<typename ServiceClass::Response>(res);

    return serialized;
  }
};
#endif