#include "temoto_resource_registrar/rr_client_base.h"

namespace temoto_resource_registrar
{
  RrClientBase::RrClientBase(const std::string &name)
      : name_(name), id_(name)
  {
  }

  std::string RrClientBase::id()
  {
    return name_;
  }

  void RrClientBase::wrappedCallback()
  {
  }
} // namespace temoto_resource_registrar