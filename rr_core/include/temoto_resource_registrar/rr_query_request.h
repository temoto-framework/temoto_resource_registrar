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

#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_QUERY_REQUEST_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_QUERY_REQUEST_H

#include <string>

namespace temoto_resource_registrar
{
  class RrQueryRequest
  {
  public:
    std::string message_;

    RrQueryRequest(const std::string &request)
        : message_(request) {}

    RrQueryRequest(const RrQueryRequest &query)
        : message_(query.message_) {}

    bool operator==(const RrQueryRequest &other) const
    {
      return message_ == other.message_;
    }

  private:
  };
} // namespace temoto_resource_registrar

#endif