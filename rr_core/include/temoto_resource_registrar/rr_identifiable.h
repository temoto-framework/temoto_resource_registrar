#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_IDENTIFIABLE_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_IDENTIFIABLE_H

#include "string"

namespace temoto_resource_registrar
{
  class Identifiable
  {
  public:
    virtual ~Identifiable(){};

    virtual std::string id() = 0;
  };
} // namespace temoto_resource_registrar

#endif