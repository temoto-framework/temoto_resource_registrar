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

#include "temoto_resource_registrar/rr_client_base.h"
#include "temoto_resource_registrar/rr_id_utils.h"
#include "temoto_resource_registrar/rr_exceptions.h"

namespace temoto_resource_registrar
{
  RrClientBase::RrClientBase(const std::string &rr, const std::string &name)
  : name_(name)
  , rr_(rr)
  , id_(IDUtils::generateServerName(rr, name))
  {
  }

  std::string RrClientBase::id() const
  {
    return id_;
  }

  void RrClientBase::setCatalog(const RrCatalogPtr &reg)
  {
    rr_catalog_ = reg;
  }

  void RrClientBase::wrappedCallback()
  {
  }

  void RrClientBase::invoke(const RrQueryBase &query) const
  {
  }

  void RrClientBase::internalStatusCallback(const std::string &request_id, const Status &status)
  {
  }

  template <class UserStatusCb>
  void RrClientBase::registerUserStatusCb(const std::string &request_id, const UserStatusCb &user_status_cb)
  {
  }

  void RrClientBase::registerUserStatusCb(const std::string &request_id, void *const &t)
  {
  }

  bool RrClientBase::hasRegisteredCb(const std::string &request_id) const
  {
    throw NotImplementedException("'hasRegisteredCb' is not implemented in the base class");
    return false;
  }

  std::vector<std::string> RrClientBase::registeredCallbackQueries() const
  {
    return status_query_ids_;
  }

  std::string RrClientBase::rr() const
  {
    return rr_;
  }
} // namespace temoto_resource_registrar