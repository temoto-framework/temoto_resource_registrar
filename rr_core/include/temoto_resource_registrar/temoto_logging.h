#ifndef TEMOTO_LOGGING_H
#define TEMOTO_LOGGING_H

#include <map>
#include <string>
#include <stack>
#include <thread>
#include <mutex>
#include <typeinfo>
#include <cstdlib>
#include <boost/core/demangle.hpp>

#include <iostream> // TODO: remove after debug finished

#define GET_NAME TEMOTO_MSG_ATTR.getNsWithSlash() + __func__
#define GET_OBJ_NAME TEMOTO_MSG_ATTR.getNsWithSlash() + boost::core::demangle(typeid(*this).name()) + "::" + __func__

#define PUSH_TRACE_CONTEXT TEMOTO_MSG_ATTR.pushParentTraceContext(GET_NAME)
#define PUSH_OBJ_TRACE_CONTEXT TEMOTO_MSG_ATTR.pushParentTraceContext(GET_OBJ_NAME)
#define PRINT_TRACE_CONTEXT TEMOTO_MSG_ATTR.printContext()

class TemotoLoggingAttributes
{
public:

  typedef std::string TraceContextType;

  TemotoLoggingAttributes()
  {
    if (const char* env_p = std::getenv("TEMOTO_NAMESPACE"))
    {
      temoto_namespace_ = std::string(env_p);
    }
  }

  void pushParentTraceContext(const TraceContextType& trace_context)
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

  const std::string& getNs() const
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

static TemotoLoggingAttributes TEMOTO_MSG_ATTR;

#endif