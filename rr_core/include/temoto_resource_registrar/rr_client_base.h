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

#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_CLIENT_BASE_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_CLIENT_BASE_H

#include "rr_catalog.h"
#include "rr_identifiable.h"
#include "rr_query_base.h"

#include <iostream>
#include <string>

namespace temoto_resource_registrar
{
  class RrClientBase : public Identifiable
  {
  public:
    RrClientBase(const std::string &name);

    virtual void wrappedCallback();

    virtual std::string id();

    void setCatalog(const RrCatalogPtr &reg);

    void invoke(const RrQueryBase &query);

  protected:
    std::string name_;
    RrCatalogPtr rr_message_registry_;

  private:
    std::string id_;
  };

} // namespace temoto_resource_registrar

#endif