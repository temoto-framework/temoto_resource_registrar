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
#include "rr_exceptions.h"
#include "rr_id_utils.h"
#include "rr_identifiable.h"
#include "rr_query_base.h"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <iostream>

namespace temoto_resource_registrar
{
  class TransactionInfo
  {
  public:
    TransactionInfo(const int &type, const RrQueryBase &query) : type_(type), base_query_(query){};
    // 100 - start transaction
    // 200 - end transaction
    int type_;
    RrQueryBase base_query_;
  };

  class RrServerBase : public Identifiable<std::string>
  {

  public:
    RrServerBase(const std::string &name, void (*loadCallback)(RrQueryBase &), void (*unLoadCallback)(RrQueryBase &))
        : name_(name), load_callback_ptr_(loadCallback), unload_callback_ptr_(unLoadCallback){};

    std::string id()
    {
      return id_;
    };

    void initializeServer(const std::string &rr, const RrCatalogPtr &reg)
    {
      id_ = IDUtils::generateServerName(rr, name_);
      rr_catalog_ = reg;
      initialize();
      initialized_ = true;
    }

    template <class Q>
    void processQuery(Q &query) const
    {
      load_callback_ptr_(query);
    };

    virtual bool unloadMessage(const std::string &id) = 0;

    void registerTransactionCb(std::function<void(const TransactionInfo &)> callback)
    {
      transaction_callback_ptr_ = callback;
    }

  protected:
    virtual void initialize(){
    };

    RrCatalogPtr rr_catalog_;
    //keeping debug values, just in case for dev

    std::string id_;
    bool initialized_ = false;

    void (*load_callback_ptr_)(RrQueryBase &);
    void (*unload_callback_ptr_)(RrQueryBase &);

    std::function<void(const TransactionInfo &)> transaction_callback_ptr_;

    std::string generateId() const
    {
      return boost::uuids::to_string(boost::uuids::random_generator()());
    }

  private:
    std::string name_;
  };

} // namespace temoto_resource_registrar

#endif