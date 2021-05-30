#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_SERVER_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_SERVER_H

#include "rclcpp/rclcpp.hpp"

#include "temoto_resource_registrar/rr_serializer.h"
#include "temoto_resource_registrar/rr_server_base.h"

#include "rr/message_serializer.h"
#include "rr/query_utils.h"
#include "rr/ros2_client.h"
#include "rr/ros2_query.h"

#include <functional>
#include <sstream>
#include <string>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief A wrapper class for the temoto_resource_registrar::RRServerBase. Provides templating to support multiple message types.
 * This class is responsible for executing resource loading and unloading related logic.
 * 
 * @tparam ServiceClass - Type of the resource this server will serve.
 */
template <class ServiceClass>
class Ros2Server : public temoto_resource_registrar::RrServerBase
{

public:
  Ros2Server(const std::string &name, std::shared_ptr<rclcpp::Node> node,
             std::function<void(const std::shared_ptr<typename ServiceClass::Request>,
                                std::shared_ptr<typename ServiceClass::Response>)>
                 member_load_cb,
             std::function<void(const std::shared_ptr<typename ServiceClass::Request>,
                                std::shared_ptr<typename ServiceClass::Response>)>
                 member_unload_cb,
             std::function<void(const std::shared_ptr<typename ServiceClass::Request>,
                                std::shared_ptr<typename ServiceClass::Response>,
                                const temoto_resource_registrar::Status &)>
                 member_status_cb = NULL)
      : temoto_resource_registrar::RrServerBase(name, NULL, NULL),
        member_load_cb_(member_load_cb),
        member_unload_cb_(member_unload_cb),
        member_status_cb_(member_status_cb),
        node_(node)
  {
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), name);
    //initialize();
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

    //ROS_INFO_STREAM("Starting to unload id: " << id);

    //ROS_INFO_STREAM("------------------ Catalog");
    rr_catalog_->print();
    //ROS_INFO_STREAM("------------------ Catalog");

    //ROS_INFO_STREAM("loading serialized request and response from catalog.. ");

    //ROS_INFO_STREAM("unloadMessage");
    temoto_resource_registrar::QueryContainer<std::string> original_container = rr_catalog_->findOriginalContainer(id);
    std::string serialized_request = original_container.raw_request_;
    //ROS_INFO_STREAM("Catalog data... isEmpty: " << original_container.empty_);
    //ROS_INFO_STREAM("Catalog data... q_.id(): " << original_container.q_.id());
    //ROS_INFO_STREAM("Catalog data... raw_query_: " << original_container.raw_query_);
    //ROS_INFO_STREAM("Catalog data... raw_request_: " << original_container.raw_request_);

    bool can_unload = false;

    std::string serialized_response = rr_catalog_->unload(id_, id, can_unload);

    std::shared_ptr<typename ServiceClass::Request> request;
    std::shared_ptr<typename ServiceClass::Response> response;

    try
    {
      request = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Request>(serialized_request);
      response = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(serialized_response);

      response->temoto_metadata.request_id = original_container.q_.id();

      //ROS_INFO_STREAM("Found response with legth: " << serialized_response.size());
    }
    catch (const temoto_resource_registrar::DeserializationException &e)
    {
      //ROS_WARN_STREAM("Request length: " << serialized_request.size());
      //ROS_WARN_STREAM("Response length: " << serialized_response.size());
      //ROS_WARN_STREAM("Some serialization/deserialization issue in server unload");
    }

    if (can_unload)
    {

      //ROS_INFO_STREAM("Resource can be unloaded. Executing callback.");
      //ROS_INFO_STREAM("req: " << request);
      //ROS_INFO_STREAM("res: " << response);

      member_unload_cb_(request, response);
    }
    else
    {
      //ROS_INFO_STREAM("Resource can not be unloaded. Printing catalog for debug");
    }

    rr_catalog_->print();
    rr_catalog_->saveCatalog();

    //ROS_INFO_STREAM("unload done for id: " << id << " result: " << (serialized_response.size() > 0));
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
  void serverCallback(const std::shared_ptr<typename ServiceClass::Request> req, std::shared_ptr<typename ServiceClass::Response> res)
  {
    //ROS_WARN_STREAM("Starting serverCallback " << id());

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ros 2 server CB gotten");

    std::shared_ptr<typename ServiceClass::Request> sanitized_req = sanityzeRequest(req);
    
    bool has_equals_defined = QueryUtils::EqualExists<typename ServiceClass::Request>::value;
    //ROS_INFO_STREAM("request has equals defined?: " << has_equals_defined);

    std::string serialized_request = temoto_resource_registrar::MessageSerializer::serializeMessage<typename ServiceClass::Request>(sanitized_req);
    std::string request_id = "";

    //ROS_INFO_STREAM("checking existance of request in catalog... ");
    //ROS_INFO_STREAM("Message: " << req);
    //ROS_INFO_STREAM("sanitized Message: " << sanitized_req);

    if (has_equals_defined)
    {
      //ROS_INFO_STREAM("evaluating uniqueness based on ==");
      std::vector<temoto_resource_registrar::QueryContainer<std::string>> all_server_requests = rr_catalog_->getUniqueServerQueries(id_);
      for (const auto &q : all_server_requests)
      {
        std::string ser_req = q.raw_request_;
        std::shared_ptr<typename ServiceClass::Request> request = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Request>(ser_req);
        if (request == sanitized_req)
        {
          serialized_request = ser_req;
          request_id = q.q_.id();
        }
      }
    }
    else
    {
      //ROS_INFO_STREAM("evaluating uniqueness based on string comparison");
      request_id = this->rr_catalog_->queryExists(id_, serialized_request);
    }

    std::string generated_id = generateId();
    res->temoto_metadata.request_id = generated_id;

    //ROS_INFO_STREAM("Generated request id: " << generated_id);

    Ros2Query<ServiceClass> wrapped_query = wrap(req, res);

    if (request_id.size() == 0)
    {

      try
      {
        //ROS_INFO("Executing query startup callback");
        transaction_callback_ptr_(temoto_resource_registrar::TransactionInfo(100, wrapped_query));

        //ROS_INFO("Query is unique. executing callback");

        member_load_cb_(req, res);

        //ROS_INFO_STREAM("Storing query in catalog...");

        rr_catalog_->storeQuery(id_,
                                wrapped_query,
                                serialized_request,
                                sanitizeAndSerialize(res));

        //ROS_INFO_STREAM("Stored!");
      }
      catch (const resource_registrar::TemotoErrorStack &e)
      {
        //ROS_WARN_STREAM("Server encountered an error while executing a callback: " << e.getMessage());
        wrapped_query.responseMetadata().errorStack().appendError(e);
        // store metadata object as serialized string in response
        res->temoto_metadata.status = 500;
        res->temoto_metadata.metadata = Serializer::serialize<temoto_resource_registrar::ResponseMetadata>(wrapped_query.responseMetadata());
      }

      //ROS_INFO("Executing query finished callback");
      transaction_callback_ptr_(temoto_resource_registrar::TransactionInfo(200, wrapped_query));
    }
    else
    {
      //ROS_INFO("Request found. No storage needed. Fetching it... ");
      std::shared_ptr<typename ServiceClass::Response> fetched_response = fetchResponse(request_id, wrapped_query);
      //ROS_INFO("Fetching done...");
      fetched_response->temoto_metadata.request_id = generated_id;
      res = fetched_response;
    }

    rr_catalog_->saveCatalog();
    //ROS_WARN_STREAM("server call end " << res.temoto_metadata.request_id << " " << id());

    
  }

  void triggerCallback(const temoto_resource_registrar::Status &status) const
  {
    //ROS_INFO_STREAM("Triggering callback logic..." << id());
    if (member_status_cb_ != NULL)
    {
      std::shared_ptr<typename ServiceClass::Request> request = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Request>(status.serialised_request_);
      std::shared_ptr<typename ServiceClass::Response> response = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(status.serialised_response_);
      member_status_cb_(request, response, status);
    }
  }

protected:
  std::function<void(const std::shared_ptr<typename ServiceClass::Request>,
                     std::shared_ptr<typename ServiceClass::Response>)>
      member_load_cb_;
  std::function<void(const std::shared_ptr<typename ServiceClass::Request>,
                     std::shared_ptr<typename ServiceClass::Response>)>
      member_unload_cb_;
  std::function<void(const std::shared_ptr<typename ServiceClass::Request>,
                     std::shared_ptr<typename ServiceClass::Response>,
                     const temoto_resource_registrar::Status &)>
      member_status_cb_;

private:
  //ros::NodeHandle nh_;
  //ros::ServiceServer service_;

  typename rclcpp::Client<ServiceClass>::SharedPtr client_;
  typename rclcpp::Service<ServiceClass>::SharedPtr service_;

  std::shared_ptr<rclcpp::Node> node_;

  

  virtual void initialize()
  {
    //ROS_INFO_STREAM("Starting up server..." << id_);
    //service_ = nh_.advertiseService(id_, &Ros2Server::serverCallback, this);
    //ROS_INFO_STREAM("Starting up server done!!!");
    //client_ = this->serverCallback<ServiceClass>(id());
    service_ = node_->create_service<ServiceClass>(id(), std::bind(&Ros2Server::serverCallback, this, _1, _2));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "initialized server with ID:"); 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), id()); 
  }

  Ros2Query<ServiceClass> wrap(std::shared_ptr<typename ServiceClass::Request> req, std::shared_ptr<typename ServiceClass::Response> res)
  {
    return Ros2Query<ServiceClass>(req->temoto_metadata, res->temoto_metadata);
  }

  std::shared_ptr<typename ServiceClass::Response> fetchResponse(const std::string &requestId, Ros2Query<ServiceClass> query) const
  {
    std::string serialized_response = rr_catalog_->processExisting(id_, requestId, query);
    std::shared_ptr<typename ServiceClass::Response> fetched_response = temoto_resource_registrar::MessageSerializer::deSerializeMessage<typename ServiceClass::Response>(serialized_response);

    return fetched_response;
  }

  static std::shared_ptr<typename ServiceClass::Request> sanityzeRequest(const std::shared_ptr<typename ServiceClass::Request> &data)
  {
    typename ServiceClass::Request empty;

    std::shared_ptr<typename ServiceClass::Request> res = std::make_shared<typename ServiceClass::Request>(*data);
    res->temoto_metadata = empty.temoto_metadata;
    return res;
  }

  std::string sanitizeAndSerialize(const std::shared_ptr<typename ServiceClass::Response> &res)
  {
    typename ServiceClass::Response empty;

    std::shared_ptr<typename ServiceClass::Response> copy = std::make_shared<typename ServiceClass::Response>(*res);
    copy->temoto_metadata = empty.temoto_metadata;

    std::string serialized = temoto_resource_registrar::MessageSerializer::serializeMessage<typename ServiceClass::Response>(copy);

    return serialized;
  }
};
#endif