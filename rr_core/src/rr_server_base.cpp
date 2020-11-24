#include "temoto_resource_registrar/rr_server_base.h"

namespace temoto_resource_registrar
{
  RrServerBase::RrServerBase(const std::string &name, void (*loadCallback)(), void (*unLoadCallback)())
      : name_(name), load_callback_ptr_(loadCallback), unload_callback_ptr_(unLoadCallback), class_name_(__func__)
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

  //RrClientBase RrServerBase::buildClient(const std::string &clientName, RrRegistryPtr rr_registry)
  //{
  //  return RrClientBase(clientName, rr_registry);
  //}

  std::string RrServerBase::id()
  {
    return name_;
  }

  void RrServerBase::loadResource()
  {
    load_callback_ptr_();
  }
} // namespace temoto_resource_registrar