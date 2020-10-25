#include "temoto_resource_registrar/rr_base.h"
#include <iostream>

namespace temoto_resource_registrar
{
    RrBase::RrBase()
        : rr_registry_(std::make_shared<RrRegistry>())
    {}

    void RrBase::addServer(std::unique_ptr<RrServerBase> base_server)
    {
        rr_servers_.push_back(std::move(base_server));
    }

    bool RrBase::exists(std::unique_ptr<RrServerBase> server)
    {
        for (const auto& value : rr_servers_) 
        {
            if (value -> id() == server -> id())
            {
                return true;
            }
        }

        return false;
    }

    void RrBase::call()
    {
    }

    void RrBase::print() {
        for (const auto& server : rr_servers_)
	    {
	      server->print();
	    }
    }

}

