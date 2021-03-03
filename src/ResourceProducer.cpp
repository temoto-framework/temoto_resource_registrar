#include <iostream>

#include "rr/ros1_resource_registrar.cpp"

int main(int argc, char **argv)
{
  temoto_resource_registrar::RrBase<temoto_resource_registrar::RrServerBase, temoto_resource_registrar::RrClientBase> rr_m0("rr_m0");
  std::cout << "yup " << rr_m0.name() << std::endl;
}