#include "temoto_resource_registrar/rr_server_base.h"
#include <iostream>

namespace temoto_resource_registrar
{
  RrServerBase::RrServerBase(const std::string &name, void (*loadCallback)(), void (*unLoadCallback)())
      : name_(name), load_callback_ptr_(loadCallback), unload_callback_ptr_(unLoadCallback), class_name_(__func__), id_(calculateId())
  {
  }

  RrServerBase::RrServerBase(const std::string &name, const std::string &className, void (*loadCallback)(), void (*unLoadCallback)())
      : RrServerBase(name, loadCallback, unLoadCallback)
  {
    class_name_ = className;
  }

  void RrServerBase::print()
  {
    std::cout << "I am '" << name_ << "' (" << class_name_ << "). "
              << "My ID: " << id() << std::endl;
  }

  std::string RrServerBase::id()
  {
    return name_;
  }

  unsigned int RrServerBase::calculateId()
  {
    boost::crc_32_type crc32;
    crc32.process_bytes(name_.data(), name_.length());
    crc32.process_bytes(class_name_.data(), class_name_.length());
    return crc32.checksum();
  }
} // namespace temoto_resource_registrar