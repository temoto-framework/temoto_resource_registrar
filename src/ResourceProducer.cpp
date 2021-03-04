#include <iostream>

#include "rr/ros1_resource_registrar.h"

int main(int argc, char **argv)
{
  temoto_resource_registrar::ResourceRegistrarRos1 rr("rr_m0");
  std::cout << "yup " << rr.name() << std::endl;
}