#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_STATUS_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_STATUS_H

#include <string>

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

  class StatusMessage {
    public:
    protected:
    private:
      std::string queryId_;
      Status status_;
      std::string message_;
  };
} // namespace temoto_resource_registrar
#endif