#ifndef TEMOTO_RESOURCE_REGISTRAR_SERIALIZER_H
#define TEMOTO_RESOURCE_REGISTRAR_SERIALIZER_H

#include <iostream>

class Serializer
{
public:
  template <class SerialClass>
  static std::string serialize(const SerialClass &message)
  {
    std::stringstream ss;
    boost::archive::binary_oarchive oa(ss);
    oa << message;
    return ss.str();
  };

  template <class SerialClass>
  static SerialClass deserialize(const std::string &data)
  {
    std::stringstream ss(data);
    boost::archive::binary_iarchive ia(ss);
    SerialClass obj;
    ia >> obj;
    return obj;
  };

private:
  Serializer() {}
};
#endif