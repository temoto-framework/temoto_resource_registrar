#include <iostream>
#include "temoto_resource_registrar/rr_base.h"

int main(int argc, char** argv)
{
  temoto_resource_registrar::RrBase resource_registrar;
  std::cout << "Created a TeMoto Resource Registrar object" << std::endl;
  return 0;
}