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

#include <boost/core/demangle.hpp>
#include <cstdlib>
#include <map>
#include <mutex>
#include <stack>
#include <string>
#include <thread>
#include <typeinfo>

#include <iostream> // TODO: remove after debug finished

#define GET_NAME_FF TEMOTO_LOG_ATTR.getNsWithSlash() + __func__
#define GET_NAME TEMOTO_LOG_ATTR.getNsWithSlash() + boost::core::demangle(typeid(*this).name()) + "::" + __func__

#define PUSH_TRACE_CONTEXT_FF TEMOTO_LOG_ATTR.pushParentTraceContext(GET_NAME)
#define PUSH_TRACE_CONTEXT TEMOTO_LOG_ATTR.pushParentTraceContext(GET_OBJ_NAME)
#define PRINT_TRACE_CONTEXT TEMOTO_LOG_ATTR.printContext()

class TemotoLoggingAttributes
{
public:
  typedef std::string TraceContextType;

  TemotoLoggingAttributes()
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

  void printContext() const
  {
    std::lock_guard<std::recursive_mutex> l(trace_stacks_mtx_);
    std::stack<TraceContextType> stack_cpy = trace_stacks_.at(std::this_thread::get_id());

    while (!stack_cpy.empty())
    {
      std::cout << stack_cpy.top() << std::endl;
      stack_cpy.pop();
    }
  }

  const std::string &getNs() const
  {
    return temoto_namespace_;
  }

  std::string getNsWithSlash() const
  {
    if (!temoto_namespace_.empty())
    {
      return temoto_namespace_ + "/";
    }
    else
    {
      return std::string();
    }
  }

private:
  std::string temoto_namespace_;
  std::map<std::thread::id, std::stack<TraceContextType>> trace_stacks_;
  mutable std::recursive_mutex trace_stacks_mtx_;
};

extern TemotoLoggingAttributes TEMOTO_LOG_ATTR;

#endif