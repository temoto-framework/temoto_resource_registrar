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

    template <class ServiceClass, class Request, class Response>
    class Ros1Server : public temoto_resource_registrar::RrServerBase
{
public:
  Ros1Server(const std::string &name,
             void (*loadCallback)(Request &, Response &),
             void (*unLoadCallback)(Request &, Response &)) : temoto_resource_registrar::RrServerBase(name, NULL, NULL),
                                                              typed_load_callback_ptr_(loadCallback),
                                                              typed_unload_callback_ptr_(unLoadCallback)
  {
    ROS_INFO_STREAM("Starting up server..." << name);

    service_ = nh_.advertiseService(name, &Ros1Server::serverCallback, this);
    ROS_INFO("Starting up server done!!!");
  }

  bool unloadMessage(const std::string &id)
  {
    ROS_INFO_STREAM("Starting to unload id: " << id);

    ROS_INFO_STREAM("checkign for dependencies... ");
    std::string serializedResponse = rr_catalog_->unload(name_, id);
    Response response = MessageSerializer::deSerializeMessage<Response>(serializedResponse);
    ROS_INFO_STREAM("Unload result size: " << serializedResponse.size());

    if (rr_catalog_->canBeUnloaded(name_))
    {
      ROS_INFO_STREAM("Time for unload CB!");
    }

    return serializedResponse.size() > 0;
  }

  bool serverCallback(Request &req, Response &res)
  {
    ROS_INFO("In Server Handler!!!");

    std::string generatedId = generateId();
    res.TemotoMetadata.requestId = generatedId;
    Ros1Query<Request, Response> wrappedQuery = wrapQuery(req, res);

    std::string serializedRequest = MessageSerializer::serializeMessage<Request>(req);

    ROS_INFO_STREAM("checking existance..." << serializedRequest.size());
    std::string requestId = this->rr_catalog_->queryExists(name_, serializedRequest);

    if (requestId.size() == 0)
    {
      ROS_INFO("NOPE, does not exist");
      typed_load_callback_ptr_(req, res);

      ROS_INFO_STREAM("lets hope it has a value1: " << res.loadMessage);
      ROS_INFO_STREAM("TIME TO STORE!!!");

      rr_catalog_->storeQuery(name_,
                              wrappedQuery,
                              serializedRequest,
                              sanitizeAndSerialize(res));

      ROS_INFO_STREAM("STORED!!!");
    }
    else
    {
      ROS_INFO("Request found. No storage needed. Fetching it... ");
      Response fetchedResponse = fetchResponse(requestId, wrappedQuery);
      ROS_INFO("Fetching done...");
      res = fetchedResponse;
    }

    return true;
  }

protected:
  void (*typed_load_callback_ptr_)(Request &, Response &);
  void (*typed_unload_callback_ptr_)(Request &, Response &);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;

  void storeQuery(const std::string &rawRequest, Ros1Query<Request, Response> query) const
  {
    rr_catalog_->storeQuery(name_,
                            query,
                            rawRequest,
                            sanitizeAndSerialize(query.response()));
  }

  Response fetchResponse(const std::string &requestId, Ros1Query<Request, Response> query) const
  {
    std::string serializedResponse = rr_catalog_->processExisting(name_, requestId, query);
    Response fetchedResponse = MessageSerializer::deSerializeMessage<Response>(serializedResponse);
    fetchedResponse.TemotoMetadata.requestId = query.id();

    return fetchedResponse;
  }

  template <class SanitizeClass>
  static SanitizeClass sanityzeRequest(SanitizeClass data)
  {
    SanitizeClass empty;
    data.TemotoMetadata = empty.TemotoMetadata;
    return data;
  }

  Ros1Query<Request, Response> wrapQuery(Request req, Response res)
  {
    Ros1Query<Request, Response> q(req, sanityzeRequest<Response>(res));
    q.setId(res.TemotoMetadata.requestId);
    return q;
  }

  std::string sanitizeAndSerialize(Response res) {
    std::string serialized = MessageSerializer::serializeMessage<Response>(sanityzeRequest<Response>(res));

    return serialized;
  }
  
};
#endif