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

#ifndef TEMOTO_RESOURCE_REGISTRAR__TEMOTO_ERROR_H
#define TEMOTO_RESOURCE_REGISTRAR__TEMOTO_ERROR_H

#include <vector>
#include <string>
#include <exception>
#include <time.h>
#include <sstream>
#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include "temoto_resource_registrar/temoto_logging.h"

#define TEMOTO_ERROR_FF(message) resource_registrar::TemotoError(message, GET_NAME_FF)
#define TEMOTO_ERROR_(message) resource_registrar::TemotoError(message, GET_NAME)

#define TEMOTO_ERRSTACK_FF(message) resource_registrar::TemotoErrorStack(message, GET_NAME_FF)
#define TEMOTO_ERRSTACK(message) resource_registrar::TemotoErrorStack(message, GET_NAME)

#define FWD_TEMOTO_ERRSTACK_FF(error_stack) error_stack.appendError("forwarding", GET_NAME_FF)
#define FWD_TEMOTO_ERRSTACK(error_stack) error_stack.appendError("forwarding", GET_NAME)

namespace resource_registrar
{
class TemotoError : public std::exception
{
public:
  TemotoError()
  {}

  TemotoError(std::string message, std::string origin)
  : time_(time(NULL))
  , message_(message)
  , origin_(origin)
  {}

  TemotoError(unsigned int time, std::string message, std::string origin)
  : time_(time)
  , message_(message)
  , origin_(origin)
  {}

  TemotoError(const TemotoError& te)
  : time_(te.time_)
  , message_(te.message_)
  , origin_(te.origin_)
  {}

  const unsigned long& getTime() const
  {
    return time_;
  }

  const std::string& getMessage() const
  {
    return message_;
  }

  const std::string& getOrigin() const
  {
    return origin_;
  }

  virtual const char* what() const noexcept
  {
    return getMessage().c_str();
  }

protected:
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /* version */)
  {
    ar &origin_ &message_ &time_;
  }

private:
  unsigned long time_;
  std::string message_;
  std::string origin_;
};

class TemotoErrorStack : public std::exception
{
public:
  TemotoErrorStack()
  {}

  TemotoErrorStack(std::string error_message, std::string origin)
  : error_stack_({TemotoError(error_message, origin)})
  {}

  TemotoErrorStack(const TemotoErrorStack& tes)
  : error_stack_(tes.error_stack_)
  , messages_(tes.messages_)
  {}

  TemotoErrorStack(const std::string &serialized_errstack)
  {
    std::stringstream ss(serialized_errstack);
    boost::archive::binary_iarchive ia(ss);
    ia >> *this;
  }

  TemotoErrorStack appendError(std::string error_message, std::string origin)
  {
    error_stack_.emplace_back(error_message, origin);
    return *this;
  }

  void appendError(const TemotoErrorStack& tes)
  {
    error_stack_.insert(error_stack_.end(), tes.getErrorStack().begin(), tes.getErrorStack().end());
  }

  const std::string& getMessage() const
  {
    messages_.clear();
    
    for (unsigned int i=0; i<error_stack_.size(); i++)
    {
      if (i==0)
      {
        messages_ += "origin [in " + error_stack_.at(i).getOrigin() + "]: ";
        messages_ += error_stack_.at(i).getMessage() + "\n";
      }
      else
      {
        messages_ += "   fwd [in " + error_stack_.at(i).getOrigin() + "]: ";
        messages_ += error_stack_.at(i).getMessage() + "\n";
      }
    }
    return messages_;
  }

  const std::vector<TemotoError>& getErrorStack() const
  {
    return error_stack_;
  }

  virtual const char* what() const noexcept
  {
    return getMessage().c_str();
  }

  const TemotoError front()
  {
    return error_stack_.front();
  }

  std::string serialize()
  {
    std::stringstream ss;
    boost::archive::binary_oarchive oa(ss);
    oa << *this;
    return ss.str();
  }

protected:
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /* version */)
  {
    ar &error_stack_;
  }

private:
  std::vector<TemotoError> error_stack_;
  mutable std::string messages_;
};
} // resource registrar namespace

#endif