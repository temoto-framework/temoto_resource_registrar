#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_STATUS_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_STATUS_H

#include <string>

namespace temoto_resource_registrar
{

  struct Status
  {

    enum class State
    {
      OK,
      UPDATE,
      WARNING,
      ERROR,
      FATAL
    };

    State state_;
    std::string id_;
    std::string message_;
  };
} // namespace temoto_resource_registrar
#endif