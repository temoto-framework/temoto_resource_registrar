#include "ros/ros.h"

#include "temoto_resource_registrar/rr_server_base.h"

template <class ServiceClass, class Request, class Response>
class Ros1Server : public temoto_resource_registrar::RrServerBase
{
public:
  Ros1Server(const std::string &name,
             void (*loadCallback)(ServiceClass &),
             void (*unLoadCallback)(ServiceClass &)) : temoto_resource_registrar::RrServerBase(name, NULL, NULL),
                                                       typed_load_callback_ptr_(loadCallback),
                                                       typed_unload_callback_ptr_(unLoadCallback)
  {
    ROS_INFO_STREAM("Starting up server..." << name);

    service_ = nh_.advertiseService(name, &Ros1Server::serverCallback, this);
    ROS_INFO("Starting up server done!!!");
  }

  bool unloadMessage(const std::string &id)
  {
    return false;
  }

  bool serverCallback(Request &req, Response &res)
  {
    ROS_INFO("In Server Handler!!!");

    std::string generatedId = generateId();
    res.TemotoMetadata.requestId = generatedId;

    

    namespace ser = ros::serialization;

    uint32_t serial_size = ser::serializationLength(req);

    ROS_INFO_STREAM("REQUEST TARGET: " << req.loadTarget << serial_size) ;
    
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ser::OStream stream(buffer.get(), serial_size);

    ser::serialize(stream, req);

    ROS_INFO_STREAM("Serialized request len: " << stream.getLength() << " - ");

    std::string output;
    output.resize(stream.getLength());
    memcpy(stream.getData(), &output[0], stream.getLength());

    

    //std::string requestId = rr_catalog_->queryExists(name_, serializedRequest);

    return true;
  }

protected:
  void (*typed_load_callback_ptr_)(ServiceClass &);
  void (*typed_unload_callback_ptr_)(ServiceClass &);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;
};