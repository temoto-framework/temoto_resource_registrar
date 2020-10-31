#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <temoto_resource_registrar/rr_server_base.h>



template <class T>
class ServerDerivedRos : public temoto_resource_registrar::RrServerBase
{

public:
    ServerDerivedRos(const std::string& name, void (*loadCallback)(), void (*unLoadCallback)())
        : temoto_resource_registrar::RrServerBase(name, __func__)
        , loadCallbackPtr(loadCallback)
        , unLoadCallbackPtr(unLoadCallback)
    {
        std::string serviceName = this->name_;

        ros::NodeHandle nHandler;

        ros::AdvertiseServiceOptions load_service_opts =
            ros::AdvertiseServiceOptions::create<T>(
                serviceName,
                boost::bind(&ServerDerivedRos<T>::wrappedLoadCallback, this, _1, _2),
                ros::VoidPtr(), 
                &this->load_cb_queue_);

        ros_server_ = nHandler.advertiseService(load_service_opts);

        ROS_INFO("LOADED? I HOPE SO!");

        ros::spin();
    }

private:

    ros::ServiceServer ros_server_;
    ros::CallbackQueue load_cb_queue_;

    void (*loadCallbackPtr)();
    void (*unLoadCallbackPtr)();

    bool wrappedLoadCallback(typename T::Request& req, typename T::Response& res)
    {
        return true;
    }
};
