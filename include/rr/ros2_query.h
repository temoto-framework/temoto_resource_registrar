#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_QUERY_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_QUERY_H

#include "temoto_resource_registrar/rr_query_base.h"
#include "temoto_resource_registrar/rr_serializer.h"
#include "tutorial_interfaces/msg/temoto_request_metadata.hpp"
#include "tutorial_interfaces/msg/temoto_response_metadata.hpp"

/**
 * @brief A wrapper class for temoto_resource_registrar::RrQueryBase. Used as a intermediery container to hande data by the ResourceRegistrar.
 * 
 * @tparam ServiceClass - Type of the server, a ROS1 srv
 */
template <class ServiceClass>
class Ros2Query : public temoto_resource_registrar::RrQueryBase
{
public:
  Ros2Query() {}

  /**
   * @brief Construct a new Ros 1 Query object. Does not store existing field valus of the request.
   * 
   * @param metadata 
   */
  Ros2Query(const tutorial_interfaces::msg::TemotoRequestMetadata &request_data,
            const tutorial_interfaces::msg::TemotoResponseMetadata &response_data)
  {
    setRr(request_data.serving_rr);
    setOrigin(request_data.origin_rr);

    setId(response_data.request_id);
    setStatus(response_data.status);

    if (response_data.metadata.size() > 0)
      setMetadata(Serializer::deserialize<temoto_resource_registrar::QueryMetadata>(response_data.metadata));
  }

  /**
   * @brief Construct a new Ros 1 Query object. Stores existing request data.
   * 
   * @param query 
   */
  Ros2Query(const ServiceClass &query) : Ros2Query(query.request.temoto_metadata, query.response.temoto_metadata)
  {
    typed_query_ = query;
  }

  /**
   * @brief Returs the reference of the stored srv request.
   * 
   * @return ServiceClass::Request 
   */
  typename ServiceClass::Request request()
  {
    return typed_query_.request;
  }

  /**
 * @brief Returs the reference of the stored srv response.
 * 
 * @return ServiceClass::Response 
 */
  typename ServiceClass::Response response()
  {
    return typed_query_.response;
  }

  /**
   * @brief Modifies an existing request and response with metadata from the resourecRegistrar.
   * 
   * @param req - ServiceClass::Request that will be modified
   * @param res - ServiceClass::Response that will be modified
   */
  void rosQuery(typename ServiceClass::Request &req, typename ServiceClass::Response &res)
  {
    ServiceClass sq = rosQuery();

    req.temoto_metadata = sq.request.temoto_metadata;
    res.temoto_metadata = sq.response.temoto_metadata;
  }

  /**
   * @brief Returns a new ServiceClass type object that has its metadata filled.
   * 
   * @return ServiceClass 
   */
  ServiceClass rosQuery()
  {
    //ROS_INFO_STREAM("Returning ROS query");

    typed_query_.request.temoto_metadata.serving_rr = rr();
    typed_query_.request.temoto_metadata.origin_rr = origin();

    typed_query_.response.temoto_metadata.request_id = id();
    typed_query_.response.temoto_metadata.status = status();
    typed_query_.response.temoto_metadata.metadata = Serializer::serialize<temoto_resource_registrar::QueryMetadata>(metadata());

    return typed_query_;
  }

protected:
  ServiceClass typed_query_;

private:
};
#endif