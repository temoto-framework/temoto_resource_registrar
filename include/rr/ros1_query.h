#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_QUERY_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_QUERY_H

#include "temoto_resource_registrar/rr_query_container.h"

/**
 * @brief A wrapper class for temoto_resource_registrar::RrQueryBase. Used as a intermediery container to hande data by the ResourceRegistrar.
 * 
 * @tparam ServiceClass - Type of the server, a ROS1 srv
 */
template <class ServiceClass>
class Ros1Query : public temoto_resource_registrar::RrQueryBase
{
public:
  Ros1Query() {}

  /**
   * @brief Construct a new Ros 1 Query object. Takes in a ROS1 srv type request and response object. 
   * Also populates meta-fields used by the ResourceRegistrar, like message id, servingRr, originRr and dependencies.
   * 
   * @param req 
   * @param res 
   */
  Ros1Query(const typename ServiceClass::Request &req, const typename ServiceClass::Response &res) : typed_request_(req), typed_response_(res)
  {
    setId(res.TemotoMetadata.requestId);
    setRr(res.TemotoMetadata.servingRr);
    setOrigin(res.TemotoMetadata.originRr);

    std::string delimiter = ";;";

    ROS_INFO_STREAM("Ros1Query constructor");
    for (const auto &el : res.TemotoMetadata.dependencies)
    {
      std::vector<std::string> splitDep = splitString(el, delimiter);
      ROS_INFO_STREAM(splitDep.at(0) << " - " << splitDep.at(1));
      //includeDependency(splitDep);
    }

      
  }

  /**
   * @brief Returs the reference of the stored srv request.
   * 
   * @return ServiceClass::Request& 
   */
  typename ServiceClass::Request &request()
  {
    return typed_request_;
  }

  /**
 * @brief Returs the reference of the stored srv response.
 * 
 * @return ServiceClass::Response& 
 */
  typename ServiceClass::Response &response()
  {
    return typed_response_;
  }

protected:
  typename ServiceClass::Request typed_request_;
  typename ServiceClass::Response typed_response_;

private:
  std::vector<std::string> splitString(std::string string, const std::string &splitter) {
    std::vector<std::string> res;

    size_t pos = 0;
    std::string token;

    while ((pos = string.find(splitter)) != std::string::npos)
    {
      token = string.substr(0, pos);
      res.push_back(token);
      string.erase(0, pos + splitter.length());
    }
    return res;
  }
};
#endif