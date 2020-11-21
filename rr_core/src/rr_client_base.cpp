#include "temoto_resource_registrar/rr_client_base.h"

namespace temoto_resource_registrar
{
  RrClientBase::RrClientBase(const std::string &name, RrRegistryPtr rr_registry)
      : rr_registry_(rr_registry), name_(name), id_(calculateId())
  {
  }

  std::string RrClientBase::id()
  {
    return name_;
  }

  unsigned int RrClientBase::calculateId()
  {
    boost::crc_32_type crc32;
    crc32.process_bytes(name_.data(), name_.length());
    return crc32.checksum();
  }

  void RrClientBase::wrappedCallback()
  {
  }
} // namespace temoto_resource_registrar