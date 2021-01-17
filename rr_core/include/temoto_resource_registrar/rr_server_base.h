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

#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_SERVER_BASE_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_SERVER_BASE_H

#include "rr_catalog.h"
#include "rr_client_base.h"
#include "rr_identifiable.h"
#include "rr_query_base.h"

#include <iostream>

namespace temoto_resource_registrar
{
  class RrServerBase : public Identifiable
  {

  public:
    RrServerBase(const std::string &name, void (*loadCallback)(RrQueryBase *), void (*unLoadCallback)(RrQueryBase *));

    RrServerBase(const std::string &name, const std::string &className, void (*loadCallback)(RrQueryBase *), void (*unLoadCallback)(RrQueryBase *));

    void wrappedCallback();

    virtual void print();

    std::string id();

    virtual RrQueryResponse processQuery(RrQueryBase *query);

    void setCatalog(const RrCatalogPtr &reg);

    void (*load_callback_ptr_)(RrQueryBase *);
    void (*unload_callback_ptr_)(RrQueryBase *);

  protected:
    RrCatalogPtr rr_message_registry_;
    //keeping debug values, just in case for dev
    std::string name_;
    std::string class_name_;

  private:
    unsigned int id_;
  };

} // namespace temoto_resource_registrar

#endif