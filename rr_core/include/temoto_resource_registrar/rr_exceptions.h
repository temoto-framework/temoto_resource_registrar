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

#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_EXCEPTIONS_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_EXCEPTIONS_H

#include <exception>
#include <string>

namespace temoto_resource_registrar
{
  class DeserializationException : public std::exception
  {
  public:
    DeserializationException(const char *msg) : error_message_(msg) {}

    virtual const char *what() const throw()
    {
      return error_message_.c_str();
    }

  protected:
    std::string error_message_;
  private:
  };

  class ElementNotFoundException : public std::exception
  {
  public:
    ElementNotFoundException(const char *msg) : error_message_(msg) {}

    virtual const char *what() const throw()
    {
      return error_message_.c_str();
    }

  protected:
    std::string error_message_;
  private:
  };
} // namespace temoto_resource_registrar
#endif