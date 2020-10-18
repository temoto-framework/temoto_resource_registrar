#include "ros/ros.h"
#include <temoto_resource_registrar/rr_base.h>

class ResourceRegistrarWrapper 
{
    public:
        ResourceRegistrarWrapper() 
        {
            //temoto_resource_registrar::RrBase resource_registrar;
        }
};

bool init_rr(){
    ResourceRegistrarWrapper();
    return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "temoto_rr");
    ros::NodeHandle nh;

    init_rr();
    ROS_INFO("ready for RR to do things");

    ros::spin();
    return 0;
}