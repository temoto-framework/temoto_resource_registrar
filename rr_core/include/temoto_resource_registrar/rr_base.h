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

#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_BASE_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_BASE_H

#include <vector>
#include "rr_server_base.h"
#include "rr_client_base.h"
#include "rr_registry.h"

namespace temoto_resource_registrar
{
class RrBase
{
public:
  RrBase();
  
  void addServer(std::unique_ptr<RrServerBase> base_server);

  bool exists(std::unique_ptr<RrServerBase> server);

  void call();

  void print();

private:
  std::vector<std::unique_ptr<RrServerBase>> rr_servers_;
  std::vector<std::unique_ptr<RrClientBase>> rr_clients_;
  RrRegistryPtr rr_registry_;
};

} // namespace temoto_resource_registrar

#endif