/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2020 TeMoto Telerobotics
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_QUERY_BASE_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_QUERY_BASE_H

#include "rr_query_request.h"
#include "rr_query_response.h"
#include "temoto_error.h"
#include <boost/serialization/access.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <iostream>
#include <string>
#include <unordered_map>

namespace temoto_resource_registrar
{
  class ResponseMetadata
  {
  public:

    resource_registrar::TemotoErrorStack& errorStack() { return error_stack_; }

  protected:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* version */)
    {
      ar &error_stack_;
    }

  private:
    resource_registrar::TemotoErrorStack error_stack_;
  };

  class RequestMetadata
  {
  public:
    typedef std::unordered_map<std::string, std::string> SpanContextType;

    SpanContextType &getSpanContext() { return span_context_; }

    void setSpanContext(SpanContextType span_context) { span_context_ = span_context; }

  protected:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* version */)
    {
      ar &span_context_;
    }

  private:
    resource_registrar::TemotoErrorStack error_stack_;
    SpanContextType span_context_;
  };

  class RrQueryBase
  {
  public:
    virtual ~RrQueryBase() = default;

    template <class RequestClass>
    RequestClass request() const;

    template <class ResponseClass>
    void storeResponse(ResponseClass &resp);

    void setId(const std::string &id) { requestId_ = id; };
    std::string id() const { return requestId_; }

    void setRr(const std::string &rr) { servingRr_ = rr; }
    std::string rr() { return servingRr_; }

    void setOrigin(const std::string &rr) { originRr_ = rr; }
    std::string origin() { return originRr_; }

    void setStatus(const int &status) { status_ = status; }
    int status() { return status_; }

    void setRequestMetadata(RequestMetadata metadata) { request_metadata_ = metadata; };
    RequestMetadata &requestMetadata() { return request_metadata_; }

    void setResponseMetadata(ResponseMetadata metadata) { response_metadata_ = metadata; };
    ResponseMetadata &responseMetadata() { return response_metadata_; }

  protected:
    std::string requestId_;
    std::string servingRr_;
    std::string originRr_;

    int status_;

    RequestMetadata request_metadata_;
    ResponseMetadata response_metadata_;

    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* version */)
    {
      ar &requestId_ &servingRr_ &originRr_ &status_ &request_metadata_ &response_metadata_;
    }

  private:
  };

} // namespace temoto_resource_registrar

#endif