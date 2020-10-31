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

#include "rr_registry.h"
#include "rr_query_base.h"
#include <boost/crc.hpp>

namespace temoto_resource_registrar
{

enum class State : int
{
  UNINITIALIZED = 0,
  INITIALIZED = 1
};

class RrServerBase
{

public:

  RrServerBase();

  RrServerBase(RrRegistryPtr rr_registry);

  RrServerBase(const std::string& name, const std::string& class_name);

  RrServerBase(const std::string& name, const std::string& class_name, RrRegistryPtr rr_registry);

  void state(const State& state);

  State state();

  void wrappedCallback();

  virtual void print();

  virtual unsigned int id();

protected:

  RrRegistryPtr rr_registry_;
  //keeping debug values, just in case for dev
  std::string name_;
  std::string class_name_;
  State state_ = State::UNINITIALIZED; 

private:
  unsigned int id_;
  unsigned int calculateId();

};

} // namespace temoto_resource_registrar

#endif