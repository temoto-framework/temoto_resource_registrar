#include "ros/ros.h"
#include <temoto_resource_registrar/rr_server_base.h>


template <class T>
class ServerDerivedRos : public temoto_resource_registrar::RrServerBase
{

public:
    ServerDerivedRos(const std::string& name, void (*loadCallback)(), void (*unLoadCallback)())
        : temoto_resource_registrar::RrServerBase(name, __func__)
        , loadCallbackPtr(loadCallback)
        , unLoadCallbackPtr(unLoadCallback)
    {}

    long id() override
    {
        boost::crc_32_type crc32 = crc();
        return crc32.checksum();
    }

private:

    ros::ServiceServer ros_server_;
    void (*loadCallbackPtr)();
    void (*unLoadCallbackPtr)();
};
