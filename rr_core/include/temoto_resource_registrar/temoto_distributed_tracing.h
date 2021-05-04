/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2021 TeMoto Telerobotics
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

#ifndef TEMOTO_RESOURCE_REGISTRAR__TEMOTO_DISTRIBUTED_TRACING_H
#define TEMOTO_RESOURCE_REGISTRAR__TEMOTO_DISTRIBUTED_TRACING_H

#include <thread>
#include <functional>
#include <fstream>
#include <map>
#include <stack>
#include <unordered_map>
#include "opentracing/dynamic_load.h"
#include <text_map_carrier.h>
#include "yaml-cpp/yaml.h"

namespace temoto_logging
{

struct SpanHandle; // fowrward declaration
typedef std::unordered_map<std::string, std::string> StringMap;
typedef StringMap SpanContextType;
typedef std::map<std::thread::id, std::stack<SpanHandle>> SpanStacks;

/**
 * @brief Helper datastructure to contain a span and its context
 * 
 */
struct SpanHandle
{
  std::unique_ptr<opentracing::Span> span;
  SpanContextType context;
};

/**
 * @brief Helps to clean up tracing spans before going out of scope
 * 
 */
class SpanCollector
{
public:
  SpanCollector(std::function<void()> span_cleanup_cb)
  : span_cleanup_cb_(span_cleanup_cb)
  {}

  ~SpanCollector()
  {
    span_cleanup_cb_();
  }
private:
  std::function<void()> span_cleanup_cb_;
};

} // temoto_logging namespace

#endif