#include "temoto_resource_registrar/rr_server_base.h"
#include <iostream>


namespace temoto_resource_registrar
{

    RrServerBase::RrServerBase()
        :name_("NONAME")
        , class_name_(__func__)
    {}

    RrServerBase::RrServerBase(RrRegistryPtr rr_registry)
        : rr_registry_(rr_registry)
        , state_(State::INITIALIZED)
    {}

    RrServerBase::RrServerBase(const std::string& name, const std::string& class_name)
        : name_(name)
        , class_name_(class_name)
    {}

    RrServerBase::RrServerBase(const std::string& name, const std::string& class_name, RrRegistryPtr rr_registry)
        : name_(name)
        , class_name_(class_name)
        , rr_registry_(rr_registry)
        , state_(State::INITIALIZED)
    {}

    void RrServerBase::print()
    {
        std::cout << "I am '" << name_ << "' (" << class_name_ << "). "
        << "My state is " << static_cast<std::underlying_type<State>::type>(state_)
        << "My ID: " << id() << std::endl;
    }

    void RrServerBase::state(const State& state)
    {
        state_ = state;
    }

    State RrServerBase::state()
    {
        return state_;
    }

    long RrServerBase::id()
    {
        return crc().checksum();
    }

    boost::crc_32_type& RrServerBase::crc() {
        crc32.reset();
        crc32.process_bytes(name_.data(), name_.length());
        crc32.process_bytes(class_name_.data(), class_name_.length());

        return crc32;
    }
}