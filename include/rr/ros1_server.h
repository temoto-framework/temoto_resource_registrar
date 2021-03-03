#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_SERVER_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_SERVER_H

#include "ros/ros.h"

#include "temoto_resource_registrar/rr_server_base.h"

#include "rr/message_serializer.h"
#include "rr/ros1_client.h"
#include "rr/ros1_query.h"

#include <sstream>
#include <string>
#include <vector>

    template <class ServiceClass, class Request, class Response>
    class Ros1Server : public temoto_resource_registrar::RrServerBase
{
public:
  Ros1Server(const std::string &name,
             void (*loadCallback)(Request &, Response &),
             void (*unLoadCallback)(Request &, Response &));

  bool unloadMessage(const std::string &id);

  bool serverCallback(Request &req, Response &res);

protected:
  void (*typed_load_callback_ptr_)(Request &, Response &);
  void (*typed_unload_callback_ptr_)(Request &, Response &);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;

  void storeQuery(const std::string &rawRequest, Ros1Query<Request, Response> query) const;
  Response fetchResponse(const std::string &requestId, Ros1Query<Request, Response> query) const;

  template <class SanitizeClass>
  static SanitizeClass sanityzeRequest(SanitizeClass data)
  {
    SanitizeClass empty;
    data.TemotoMetadata = empty.TemotoMetadata;
    return data;
  }

  Ros1Query<Request, Response> wrapQuery(Request req, Response res);

  std::string sanitizeAndSerialize(Response res) {
    std::string serialized = MessageSerializer::serializeMessage<Response>(sanityzeRequest<Response>(res));

    return serialized;
  }
  
};
#endif