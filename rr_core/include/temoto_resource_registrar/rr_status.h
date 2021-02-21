#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_STATUS_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_STATUS_H

namespace temoto_resource_registrar
{
  enum class Status
  {
    OK,
    UPDATE,
    WARNING,
    ERROR,
    FATAL
  };
} // namespace temoto_resource_registrar
#endif