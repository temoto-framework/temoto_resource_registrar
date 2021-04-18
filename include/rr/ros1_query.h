#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_QUERY_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_QUERY_H

#include "temoto_resource_registrar/TemotoRequestMetadata.h"
#include "temoto_resource_registrar/TemotoResponseMetadata.h"
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
   * @brief Construct a new Ros 1 Query object. Does not store existing field valus of the request.
   * 
   * @param metadata 
   */
  Ros1Query(const temoto_resource_registrar::TemotoRequestMetadata &requestData,
            const temoto_resource_registrar::TemotoResponseMetadata &responseData)
  {
    setId(responseData.requestId);
    setRr(requestData.servingRr);
    setOrigin(requestData.originRr);
    setStatus(responseData.status);

    for (const auto &el : responseData.dependencies)
    {
      std::vector<std::string> splitDep = splitString(el, DELIMITER);
      if (splitDep.size() == 2)
      {
        includeDependency(splitDep.at(0), splitDep.at(1));
      }
      else
      {
        ROS_INFO_STREAM(splitDep.size() << " " << splitDep.at(0));
      }
    }
  }

  /**
   * @brief Construct a new Ros 1 Query object. Stores existing request data.
   * 
   * @param query 
   */
  Ros1Query(const ServiceClass &query) : typed_query_(query)
  {

    setId(typed_query_.response.temotoMetadata.requestId);
    setRr(typed_query_.request.temotoMetadata.servingRr);
    setOrigin(typed_query_.request.temotoMetadata.originRr);

    for (const auto &el : typed_query_.response.temotoMetadata.dependencies)
    {
      std::vector<std::string> splitDep = splitString(el, DELIMITER);
      if (splitDep.size() == 2)
      {
        includeDependency(splitDep.at(0), splitDep.at(1));
      }
      else
      {
        ROS_INFO_STREAM(splitDep.size() << " " << splitDep.at(0));
      }
    }
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

    req.temotoMetadata = sq.request.temotoMetadata;
    res.temotoMetadata = sq.response.temotoMetadata;
  }

  /**
   * @brief Returns a new ServiceClass type object that has its metadata filled.
   * 
   * @return ServiceClass 
   */
  ServiceClass rosQuery()
  {
    typed_query_.response.temotoMetadata.requestId = id();
    typed_query_.response.temotoMetadata.dependencies = convertDependencies();
    typed_query_.response.temotoMetadata.status = status();

    typed_query_.request.temotoMetadata.servingRr = rr();
    typed_query_.request.temotoMetadata.originRr = origin();

    return typed_query_;
  }

protected:
  ServiceClass typed_query_;

private:
  std::string DELIMITER = ";;";

  std::vector<std::string> convertDependencies()
  {
    std::vector<std::string> output;
    for (const auto &dep : dependencies())
    {
      output.push_back(dep.second + DELIMITER + dep.first);
    }

    return output;
  }

  std::vector<std::string> splitString(std::string string, const std::string &splitter)
  {
    std::vector<std::string> res;

    size_t pos = 0;
    std::string token;

    while ((pos = string.find(splitter)) != std::string::npos)
    {
      token = string.substr(0, pos);
      res.push_back(token);
      string.erase(0, pos + splitter.length());
    }
    res.push_back(string);
    return res;
  }
};
#endif