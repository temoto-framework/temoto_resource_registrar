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
#include <string>

namespace temoto_resource_registrar
{
  class RrQueryBase
  {
  public:
    RrQueryBase(RrQueryRequest &request)
        : request_(request){};

    RrQueryRequest request()
    {
      return request_;
    };

    RrQueryResponse response()
    {
      return response_;
    };

    void updateResponse(RrQueryResponse resp)
    {
      response_ = resp;
    };

  private:
    RrQueryRequest request_;
    RrQueryResponse response_;
  };

} // namespace temoto_resource_registrar

#endif