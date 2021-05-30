#ifndef TEMOTO_RESOURCE_REGISTRAR__ROS_SERIALIZER_H
#define TEMOTO_RESOURCE_REGISTRAR__ROS_SERIALIZER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rcutils/allocator.h"

#include <temoto_resource_registrar/rr_exceptions.h>

namespace temoto_resource_registrar
{

  /**
 * @brief Utility class used for ROS 1 message serialization and deserialization.
 * 
 */
  class MessageSerializer
  {
  public:
    /**
   * @brief The serialization method for messages takes in a ROS 1 type message and returns a serialized string of said message.
   * ROS serialization (ros::serialization) and deserialization mechanisms are used for this.
   * 
   * @tparam PayloadType - type of the message being serialized
   * @param payload - a message of type PayloadType
   * @return std::string - string of the serialized message
   */
    template <class PayloadType>
    static std::string serializeMessage(const std::shared_ptr<PayloadType> &payload)
    {
      rclcpp::SerializedMessage serialized_msg;
      static rclcpp::Serialization<PayloadType> serializer;
      serializer.serialize_message(payload.get(), &serialized_msg);

      std::string payload_string;

      for (size_t i = 0; i < serialized_msg.size(); ++i)
      {
        payload_string += serialized_msg.get_rcl_serialized_message().buffer[i];
      }
      return payload_string;
    }

    /**
   * @brief The de-serialization method for messages takes in std::string serialized message and return the message with type PayloadType.
   * ROS serialization (ros::serialization) and deserialization mechanisms are used for this.
   * 
   * @tparam PayloadType - type of the message being de-serialized
   * @param payload_in - std::string representation of the serialized message
   * @return PayloadType -the deserialized object
   */
    template <class PayloadType>
    static std::shared_ptr<PayloadType> deSerializeMessage(std::string payload_in)
    {
      static rclcpp::Serialization<PayloadType> serializer;
      rcl_serialized_message_t ser_msg_raw;
      ser_msg_raw.buffer_length = payload_in.size();
      ser_msg_raw.buffer_capacity = ser_msg_raw.buffer_length;
      ser_msg_raw.allocator = rcl_get_default_allocator();
      ser_msg_raw.buffer = reinterpret_cast<uint8_t *>(&payload_in[0]);
      rclcpp::SerializedMessage serialized_msg(ser_msg_raw);
      auto msg = std::make_shared<PayloadType>();
      serializer.deserialize_message(&serialized_msg, msg.get());

      return msg;
    }

  private:
    MessageSerializer() {}
  };

} // namespace temoto_resource_registrar
#endif