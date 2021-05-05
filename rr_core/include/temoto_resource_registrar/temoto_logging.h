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

#ifndef TEMOTO_LOGGING__TEMOTO_LOGGING_H
#define TEMOTO_LOGGING__TEMOTO_LOGGING_H

#include <cstdlib>
#include <mutex>
#include <string>
#include <typeinfo>
#include <sstream>
#include <boost/core/demangle.hpp>
#include <console_bridge/console.h>
#include <iostream>
#include <exception>
#include "temoto_resource_registrar/string_formatting.h"

#ifdef temoto_enable_tracing
#include "temoto_resource_registrar/temoto_distributed_tracing.h"
#endif

// Logging prefix macro definitions
#define GET_NAME_FF TEMOTO_LOG_ATTR.getNsWithSlash() + __func__
#define GET_NAME TEMOTO_LOG_ATTR.getNsWithSlash() + boost::core::demangle(typeid(*this).name()) + "::" + __func__

// TeMoto logging related definitions via console bridge
#ifdef temoto_enable_tracing
  #define TEMOTO_LOG_(level, fmt, ...) \
  { \
    std::string msg = temoto_logging::format(std::string("[from " + GET_NAME_FF + "]: " + fmt).c_str(), ##__VA_ARGS__); \
    console_bridge::log(__FILE__, __LINE__, level, msg.c_str()); \
    TEMOTO_LOG_ATTR.sendSpanLog(msg); \
  }
#else
  #define TEMOTO_LOG_(level, fmt, ...) \
  { \
    std::string msg = temoto_logging::format(std::string("[from " + GET_NAME_FF + "]: " + fmt).c_str(), ##__VA_ARGS__); \
    console_bridge::log(__FILE__, __LINE__, level, msg.c_str()); \
  }
#endif

// #define TEMOTO_ERROR_(fmt, ...) \
//   TEMOTO_LOG_(console_bridge::CONSOLE_BRIDGE_LOG_ERROR, fmt, ##__VA_ARGS__)

#define TEMOTO_WARN_(fmt, ...) \
  TEMOTO_LOG_(console_bridge::CONSOLE_BRIDGE_LOG_WARN, fmt, ##__VA_ARGS__)

#define TEMOTO_INFO_(fmt, ...) \
  TEMOTO_LOG_(console_bridge::CONSOLE_BRIDGE_LOG_INFO, fmt, ##__VA_ARGS__)

#define TEMOTO_DEBUG_(fmt, ...) \
  TEMOTO_LOG_(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG, fmt, ##__VA_ARGS__)

// TeMoto logging stream related definitions via console bridge
#ifdef temoto_enable_tracing
  #define TEMOTO_LOG_STREAM_(level, fmt, ...) \
  { \
    std::stringstream ss; \
    ss << "[from " << GET_NAME_FF << "]: " << fmt; \
    std::string msg = temoto_logging::format(ss.str().c_str(), ##__VA_ARGS__); \
    console_bridge::log(__FILE__, __LINE__, level, msg.c_str()); \
    TEMOTO_LOG_ATTR.sendSpanLog(msg); \
  }
#else
  #define TEMOTO_LOG_STREAM_(level, fmt, ...) \
  { \
    std::stringstream ss; \
    ss << "[from " << GET_NAME_FF << "]: " << fmt; \
    console_bridge::log(__FILE__, __LINE__, level, ss.str().c_str(), ##__VA_ARGS__); \
  }
#endif

#define TEMOTO_ERROR_STREAM_(fmt, ...) \
  TEMOTO_LOG_STREAM_(console_bridge::CONSOLE_BRIDGE_LOG_ERROR, fmt, ##__VA_ARGS__)

#define TEMOTO_WARN_STREAM_(fmt, ...) \
  TEMOTO_LOG_STREAM_(console_bridge::CONSOLE_BRIDGE_LOG_WARN, fmt, ##__VA_ARGS__)

#define TEMOTO_INFO_STREAM_(fmt, ...) \
  TEMOTO_LOG_STREAM_(console_bridge::CONSOLE_BRIDGE_LOG_INFO, fmt, ##__VA_ARGS__)

#define TEMOTO_DEBUG_STREAM_(fmt, ...) \
  TEMOTO_LOG_STREAM_(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG, fmt, ##__VA_ARGS__)

// Distribted tracing related macro definitions
#ifdef temoto_enable_tracing
  #define START_SPAN_UNIQUE(span_collector_name) \
    temoto_logging::SpanCollector span_collector_name(TEMOTO_LOG_ATTR.startTracingSpan(GET_NAME_FF));

  #define START_SPAN START_SPAN_UNIQUE(temoto_tracing_span_collector)
#else
  #define START_SPAN_UNIQUE(span_collector_name)
  #define START_SPAN
#endif

namespace temoto_logging
{

/**
 * @brief Handles logging and distributed tracing related attributes
 * 
 */
class LoggingAttributes
{
public:
  LoggingAttributes()
  : subsystem_name_("")
  {
    if (const char *env_temoto_namespace = std::getenv("TEMOTO_NAMESPACE"))
    {
      temoto_namespace_ = std::string(env_temoto_namespace);
    }

  #ifdef temoto_enable_tracing
    if (const char *env_tracer_config_pth = std::getenv("TEMOTO_TRACER_CONFIG_PATH"))
    {
      tracer_config_path_ = std::string(env_tracer_config_pth);
    }
    else
    {
      throw std::runtime_error("TEMOTO_TRACER_CONFIG_PATH environment variable undefined");
    }
  #endif
  }

  void initialize(std::string subsystem_name = "")
  {
    subsystem_name_ = subsystem_name;
    #ifdef temoto_enable_tracing
    initializeTracer();
    #endif
  }

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

  const std::string& getSubsystemName() const
  {
    return subsystem_name_;
  }

  void setSubsystemName(const std::string& subsystem_name)
  {
    subsystem_name_ = subsystem_name;
  }

private:
  std::string temoto_namespace_;
  std::string subsystem_name_;

#ifdef temoto_enable_tracing

  SpanStacks span_stacks_;
  mutable std::recursive_mutex span_stacks_mtx_;

  opentracing::v2::expected<opentracing::v2::DynamicTracingLibraryHandle> tracer_handle_maybe_;
  opentracing::v2::expected<std::shared_ptr<opentracing::v2::Tracer>> tracer_;
  std::string tracer_config_path_;

  void initializeTracer(const std::string& tracer_lib_path = "")
  {
    //TEMOTO_DEBUG_STREAM_("Initializing the tracer");
    std::string tracer_name = temoto_namespace_ + "/" + subsystem_name_;

    /*
     * Read in the tracer's configuration.
     */ 
    std::ifstream istream{tracer_config_path_.c_str()};
    if (!istream.good()) 
    {
      throw std::runtime_error("Failed to open tracer config file: " + tracer_config_path_);
    }

    std::string tracer_config = std::string("service_name: " + tracer_name + "\n");
    std::string tracer_config_other{ 
      std::istreambuf_iterator<char>{istream}
    , std::istreambuf_iterator<char>{}};
    tracer_config += tracer_config_other;

    /*
     * Load the tracer library
     */ 
    std::string tracer_lib_str;
    if (tracer_lib_path.empty())
    {
      try
      {
        YAML::Node tracer_config_yaml = YAML::LoadFile(tracer_config_path_);
        tracer_lib_str = tracer_config_yaml["library_path"].as<std::string>();
      }
      catch(...)
      {
        throw std::runtime_error("Failed to parse tracer lib path from config: " + tracer_config_path_);
      }
    }
    else
    {
      tracer_lib_str = tracer_lib_path;
    }

    std::string error_message;
    tracer_handle_maybe_ = opentracing::DynamicallyLoadTracingLibrary(tracer_lib_str.c_str(), error_message);
    if (!tracer_handle_maybe_) 
    {
      throw std::runtime_error("Failed to load tracer library: " + error_message);
    }

    /*
     * Construct a tracer
     */ 
    tracer_ = tracer_handle_maybe_->tracer_factory().MakeTracer(tracer_config.c_str(), error_message);

    if (!tracer_) 
    {
      throw std::runtime_error("Failed to create a tracer: " + error_message);
    }
    CONSOLE_BRIDGE_logInform("[from %s] Tracer initialized", std::string(getNsWithSlash() + __func__).c_str());
  }

  void pushParentSpan(SpanHandle &span_handle)
  {
    std::lock_guard<std::recursive_mutex> l(span_stacks_mtx_);
    span_stacks_[std::this_thread::get_id()].push(std::move(span_handle));
  }

  SpanContextType topParentSpanContext() const
  {
    std::lock_guard<std::recursive_mutex> l(span_stacks_mtx_);
    return span_stacks_.at(std::this_thread::get_id()).top().context;
  }

  bool spanStackEmpty() const
  {
    std::lock_guard<std::recursive_mutex> l(span_stacks_mtx_);
    //CONSOLE_BRIDGE_logError("[from %s] spanstacks size %d", std::string(getNsWithSlash() + __func__).c_str(), span_stacks_.size());
    return span_stacks_.find(std::this_thread::get_id()) == span_stacks_.end();
  }

public:
  void popParentSpan()
  {
    std::lock_guard<std::recursive_mutex> l(span_stacks_mtx_);
    if (spanStackEmpty())
    {
      throw std::runtime_error("A pop was attempted at an empty trace span stack");
    }

    span_stacks_[std::this_thread::get_id()].pop();
    if (span_stacks_[std::this_thread::get_id()].empty())
    {
      span_stacks_.erase(std::this_thread::get_id());
    }
  }

  std::function<void()> startTracingSpan(const std::string& span_name)
  {
    std::lock_guard<std::recursive_mutex> l(span_stacks_mtx_);
    
    std::unique_ptr<opentracing::Span> tracing_span;
    if (spanStackEmpty()) 
    {
      tracing_span = (*tracer_)->StartSpan(span_name);
    }
    else
    {
      SpanContextType spc = topParentSpanContext();
      TextMapCarrier carrier(spc);
      auto span_context_maybe = (*tracer_)->Extract(carrier);
      tracing_span = (*tracer_)->StartSpan(span_name, {opentracing::ChildOf(span_context_maybe->get())});
    }

    SpanContextType span_context;
    TextMapCarrier carrier(span_context);
    auto err = (*tracer_)->Inject(tracing_span->context(), carrier); // TODO: check if successful

    SpanHandle span_handle;
    span_handle.span = std::move(tracing_span);
    span_handle.context = span_context;
    pushParentSpan(span_handle);

    return std::bind(&LoggingAttributes::popParentSpan, this);
  }

  void sendSpanLog(const std::string& msg)
  {
    std::lock_guard<std::recursive_mutex> l(span_stacks_mtx_);
    if (spanStackEmpty())
    {
      return;
    }
    span_stacks_.at(std::this_thread::get_id()).top().span->Log({{"info", msg}});
  }
#endif
};
} // temoto_logging namespace

extern temoto_logging::LoggingAttributes TEMOTO_LOG_ATTR;

#endif