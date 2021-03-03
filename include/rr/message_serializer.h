#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_SERIALIZER_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_SERIALIZER_H

#include "ros/ros.h"

#include <iostream>
#include <ros/serialization.h>

namespace ser = ros::serialization;

class MessageSerializer {
  public:
    template <class PayloadType>
    static std::string serializeMessage(const PayloadType &payload)
    {
      // Serialize the payload
      uint32_t payload_size = ros::serialization::serializationLength(payload);
      boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);
      ser::OStream stream(buffer.get(), payload_size);
      ser::serialize(stream, payload);

      // Create a string
      std::string payload_string;

      // Fill out the byte array
      for (uint32_t i = 0; i < payload_size; i++)
      {
        payload_string += (char)buffer.get()[i];
      }

      return payload_string;
    }

    template <class PayloadType>
    static PayloadType deSerializeMessage(std::string payload_in)
    {
      uint32_t payload_size = payload_in.size();
      boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);

      // Fill buffer with the serialized payload

      int counter = 0;
      for (const char byte : payload_in)
      {
        (buffer.get())[counter] = (uint32_t)byte;
        counter++;
      }

      // Convert the serialized payload to msg
      ser::IStream stream(buffer.get(), payload_size);
      PayloadType payload;
      ser::deserialize(stream, payload);
      return payload;
    }

  private:
    MessageSerializer() {}
};
#endif