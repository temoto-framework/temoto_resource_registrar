#include <iostream>

#include "rr/Ros1ResourceRegistrar.cpp"

int main(int argc, char **argv)
{
  temoto_resource_registrar::RrBase<temoto_resource_registrar::RrServerBase, temoto_resource_registrar::RrClientBase> rr_m0("rr_m0");
  std::cout << "yupp " << rr_m0.name() << std::endl;
}