#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_RESOURCE_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_RESOURCE_H

#include <string>

namespace temoto_resource_registrar
{
  class RrResource
  {
  public:
    virtual ~RrResource(){};

    virtual std::string name() = 0;
  };
} // namespace temoto_resource_registrar

#endif