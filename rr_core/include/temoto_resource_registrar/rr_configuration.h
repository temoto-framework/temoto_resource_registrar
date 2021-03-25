#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_CONFIGURATION_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_CONFIGURATION_H

#include "string"

namespace temoto_resource_registrar
{
  class Configuration
  {
  public:
    Configuration *setName(const std::string &name)
    {
      name_ = name;
      return this;
    }

    Configuration *setLocation(const std::string &location)
    {
      location_ = location;
      return this;
    }
    // TODO - implement
    Configuration *setSaveInterval(const int &interval)
    {
      save_interval_ = interval;
      return this;
    }

    Configuration *setSaveOnModify(const bool &saveOnModify)
    {
      save_on_modify_ = saveOnModify;
      return this;
    }

    Configuration *setEraseOnDestruct(const bool &eraseOnDestruct)
    {
      erase_on_destruct_ = eraseOnDestruct;
      return this;
    }

    std::string name()
    {
      return name_;
    }

    std::string location()
    {
      return location_;
    }

    int saveInterval()
    {
      return save_interval_;
    }

    bool saveOnModify()
    {
      return save_on_modify_;
    }

    bool eraseOnDestruct()
    {
      return erase_on_destruct_;
    }

  protected:
  private:
    std::string name_ = "untitled";
    std::string location_ = "./catalog.backup";
    int save_interval_ = 60;
    bool save_on_modify_ = false;
    bool erase_on_destruct_ = false;
  };
} // namespace temoto_resource_registrar

#endif