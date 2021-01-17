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

#include "temoto_resource_registrar/rr_server_base.h"

namespace temoto_resource_registrar
{
  RrServerBase::RrServerBase(const std::string &name, void (*loadCallback)(RrQueryBase *), void (*unLoadCallback)(RrQueryBase *))
      : name_(name), load_callback_ptr_(loadCallback), unload_callback_ptr_(unLoadCallback), class_name_(__func__)
  {
  }

  RrServerBase::RrServerBase(const std::string &name, const std::string &className, void (*loadCallback)(RrQueryBase *), void (*unLoadCallback)(RrQueryBase *))
      : RrServerBase(name, loadCallback, unLoadCallback)
  {
    class_name_ = className;
  }

  void RrServerBase::print()
  {
    std::cout << "I am '" << name_ << "' (" << class_name_ << "). "
              << "My ID: " << id() << std::endl;
  }

  std::string RrServerBase::id()
  {
    return name_;
  }

  RrQueryResponse RrServerBase::processQuery(RrQueryBase *query)
  {
    load_callback_ptr_(query);
  };

  void RrServerBase::setCatalog(const RrCatalogPtr &reg)
  {
    rr_message_registry_ = reg;
  };
} // namespace temoto_resource_registrar