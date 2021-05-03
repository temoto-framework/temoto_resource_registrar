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

#ifndef TEMOTO_LOGGING_H
#define TEMOTO_LOGGING_H

#include <cstdlib>
#include <map>
#include <mutex>
#include <stack>
#include <string>
#include <thread>
#include <typeinfo>
#include <boost/core/demangle.hpp>
#include <console_bridge/console.h>

#include <iostream> // TODO: remove after debug finished
#include <sstream>

// Logging prefix macro definitions
#define GET_NAME_FF TEMOTO_LOG_ATTR.getNsWithSlash() + __func__
#define GET_NAME TEMOTO_LOG_ATTR.getNsWithSlash() + boost::core::demangle(typeid(*this).name()) + "::" + __func__

// Distribted tracing related macro definitions
#define PUSH_TRACE_CONTEXT_FF TEMOTO_LOG_ATTR.pushParentTraceContext(GET_NAME)
#define PUSH_TRACE_CONTEXT TEMOTO_LOG_ATTR.pushParentTraceContext(GET_OBJ_NAME)
#define PRINT_TRACE_CONTEXT TEMOTO_LOG_ATTR.printContext()

// TeMoto logging related definitions via console bridge
#define TEMOTO_LOG_(level, fmt, ...) \
  console_bridge::log(__FILE__, __LINE__, level, std::string("[from "+GET_NAME_FF+"]: "+fmt).c_str(), ##__VA_ARGS__)

// #define TEMOTO_ERROR_(fmt, ...) \
//   TEMOTO_LOG_(console_bridge::CONSOLE_BRIDGE_LOG_ERROR, fmt, ##__VA_ARGS__)

#define TEMOTO_WARN_(fmt, ...) \
  TEMOTO_LOG_(console_bridge::CONSOLE_BRIDGE_LOG_WARN, fmt, ##__VA_ARGS__)

#define TEMOTO_INFO_(fmt, ...) \
  TEMOTO_LOG_(console_bridge::CONSOLE_BRIDGE_LOG_INFO, fmt, ##__VA_ARGS__)

#define TEMOTO_DEBUG_(fmt, ...) \
  TEMOTO_LOG_(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG, fmt, ##__VA_ARGS__)

// TeMoto logging stream related definitions via console bridge
#define TEMOTO_LOG_STREAM_(level, fmt, ...) \
{ \
  std::stringstream ss; \
  ss << "[from " << GET_NAME_FF << "]: " << fmt; \
  console_bridge::log(__FILE__, __LINE__, level, ss.str().c_str(), ##__VA_ARGS__); \
} \

#define TEMOTO_ERROR_STREAM_(fmt, ...) \
  TEMOTO_LOG_STREAM_(console_bridge::CONSOLE_BRIDGE_LOG_ERROR, fmt, ##__VA_ARGS__)

#define TEMOTO_WARN_STREAM_(fmt, ...) \
  TEMOTO_LOG_STREAM_(console_bridge::CONSOLE_BRIDGE_LOG_WARN, fmt, ##__VA_ARGS__)

#define TEMOTO_INFO_STREAM_(fmt, ...) \
  TEMOTO_LOG_STREAM_(console_bridge::CONSOLE_BRIDGE_LOG_INFO, fmt, ##__VA_ARGS__)

#define TEMOTO_DEBUG_STREAM_(fmt, ...) \
  TEMOTO_LOG_STREAM_(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG, fmt, ##__VA_ARGS__)

// #define TEMOTO_DEBUG_(fmt, ...)  \
//   console_bridge::log(__FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_DEBUG, "[" + GET_NAME + "]" + fmt, ##__VA_ARGS__)

class TemotoLoggingAttributes
{
public:
  typedef std::string TraceContextType;

  TemotoLoggingAttributes()
  : subsystem_name_("")
  {
    if (const char *env_p = std::getenv("TEMOTO_NAMESPACE"))
    {
      temoto_namespace_ = std::string(env_p);
    }
  }

  void pushParentTraceContext(const TraceContextType &trace_context)
  {
    std::lock_guard<std::recursive_mutex> l(trace_stacks_mtx_);
    trace_stacks_[std::this_thread::get_id()].push(trace_context);
  }

  TraceContextType popParentTraceContext()
  {
    std::lock_guard<std::recursive_mutex> l(trace_stacks_mtx_);
    TraceContextType trace_context = trace_stacks_[std::this_thread::get_id()].top();
    trace_stacks_[std::this_thread::get_id()].pop();
    return trace_context;
  }

  TraceContextType topParentTraceContext() const
  {
    std::lock_guard<std::recursive_mutex> l(trace_stacks_mtx_);
    return trace_stacks_.at(std::this_thread::get_id()).top();
  }

  // void printContext() const
  // {
  //   std::lock_guard<std::recursive_mutex> l(trace_stacks_mtx_);
  //   std::stack<TraceContextType> stack_cpy = trace_stacks_.at(std::this_thread::get_id());

  //   while (!stack_cpy.empty())
  //   {
  //     std::cout << stack_cpy.top() << std::endl;
  //     stack_cpy.pop();
  //   }
  // }

  const std::string &getNs() const
  {
    return temoto_namespace_;
  }

  std::string getNsWithSlash() const
  {
    std::string prefix;
    if (!temoto_namespace_.empty())
    {
      prefix = temoto_namespace_ + "/";
    }
    
    if (!subsystem_name_.empty())
    {
      prefix += subsystem_name_ + "/";
    }
    return prefix;
  }

  void setSubsystemName(const std::string& subsystem_name)
  {
    subsystem_name_ = subsystem_name;
  }

private:
  std::string temoto_namespace_;
  std::string subsystem_name_;
  std::map<std::thread::id, std::stack<TraceContextType>> trace_stacks_;
  mutable std::recursive_mutex trace_stacks_mtx_;
};

extern TemotoLoggingAttributes TEMOTO_LOG_ATTR;

#endif