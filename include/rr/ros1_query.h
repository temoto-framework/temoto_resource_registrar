#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_QUERY_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_QUERY_H

#include "temoto_resource_registrar/rr_query_container.h"

template <class RequestClass, class ResponseClass>
class Ros1Query : public temoto_resource_registrar::RrQueryBase
{
public:
  Ros1Query() {}

  Ros1Query(const RequestClass &req, const ResponseClass &res) : typed_request_(req), typed_response_(res) {
  }

  RequestClass& request()
  {
    return typed_request_;
  }

  ResponseClass& response()
  {
    return typed_response_;
  }

protected:
  RequestClass typed_request_;
  ResponseClass typed_response_;

private:
};
#endif