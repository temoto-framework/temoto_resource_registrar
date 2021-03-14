#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_STATUS_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_STATUS_H

#include <string>

namespace temoto_resource_registrar
{
  // TODO: Has to be renamed to "Status", after the enum below this struct is removed
  struct StatusTodo
  {
    enum class State
    {
      OK,
      UPDATE,
      WARNING,
      ERROR,
      FATAL
    };

    std::string message;
  };

  // TODO: Deprecated, remove this enum
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