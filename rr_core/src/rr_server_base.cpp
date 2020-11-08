#include "temoto_resource_registrar/rr_server_base.h"
#include <iostream>

namespace temoto_resource_registrar
{
  RrServerBase::RrServerBase(const std::string &name, const std::string &class_name)
      : name_(name), class_name_(class_name), id_(calculateId())
  {
  }

  RrServerBase::RrServerBase(const std::string &name, const std::string &class_name, RrRegistryPtr rr_registry)
      : name_(name), class_name_(class_name), rr_registry_(rr_registry), id_(calculateId())
  {
  }

  void RrServerBase::print()
  {
    std::cout << "I am '" << name_ << "' (" << class_name_ << "). "
              << "My ID: " << id() << std::endl;
  }

  unsigned int RrServerBase::id()
  {
    return id_;
  }

  unsigned int RrServerBase::calculateId()
  {
    boost::crc_32_type crc32;
    crc32.process_bytes(name_.data(), name_.length());
    crc32.process_bytes(class_name_.data(), class_name_.length());
    return crc32.checksum();
  }
} // namespace temoto_resource_registrar