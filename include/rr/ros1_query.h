#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_QUERY_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_QUERY_H

#include "temoto_resource_registrar/TemotoMetadata.h"
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
  Ros1Query(const temoto_resource_registrar::TemotoMetadata &metadata){
    setId(metadata.requestId);
    setRr(metadata.servingRr);
    setOrigin(metadata.originRr);

    for (const auto &el : metadata.dependencies)
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

    setId(selectNonEmpty(typed_query_.request.TemotoMetadata.requestId,
                         typed_query_.response.TemotoMetadata.requestId));
    setRr(selectNonEmpty(typed_query_.request.TemotoMetadata.servingRr,
                         typed_query_.response.TemotoMetadata.servingRr));
    setOrigin(selectNonEmpty(typed_query_.request.TemotoMetadata.originRr,
                             typed_query_.response.TemotoMetadata.originRr));

    for (const auto &el : selectLongerVector(typed_query_.request.TemotoMetadata.dependencies, typed_query_.response.TemotoMetadata.dependencies))
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

    req.TemotoMetadata = sq.request.TemotoMetadata;
    res.TemotoMetadata = sq.response.TemotoMetadata;
  }

  /**
   * @brief Returns a new ServiceClass type object that has its metadata filled.
   * 
   * @return ServiceClass 
   */
  ServiceClass rosQuery()
  {
    typed_query_.response.TemotoMetadata.requestId = id();
    typed_query_.response.TemotoMetadata.servingRr = rr();
    typed_query_.response.TemotoMetadata.originRr = origin();
    typed_query_.response.TemotoMetadata.dependencies = convertDependencies();

    typed_query_.request.TemotoMetadata.requestId = id();
    typed_query_.request.TemotoMetadata.servingRr = rr();
    typed_query_.request.TemotoMetadata.originRr = origin();
    typed_query_.request.TemotoMetadata.dependencies = convertDependencies();

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

  std::string selectNonEmpty(std::string s1, std::string s2)
  {
    if (s1.size() > s2.size())
      return s1;
    return s2;
  }

  std::vector<std::string> selectLongerVector(std::vector<std::string> v1, std::vector<std::string> v2)
  {
    if (v1.size() > v2.size())
      return v1;
    return v2;
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