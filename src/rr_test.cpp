#include <iostream>
#include "temoto_resource_registrar/rr_base.h"

int main(int argc, char** argv)
{
  /*
   * In order to test all the required features the RR must posess, we need at least 3 RR objects 
   * (named rr_m0, rr_m1 & rr_m2), each representing a "resource manager" (in terms of TeMoto terminology).
   * Dependency wise, this is what we are aiming for:
   * 
   *                                rr_m0 --> rr_m1 --> rr_m2
   * 
   * where rr_m0 is requesting for a resurce from rr_m1, which internally depends on a resource provided 
   * by rr_m2. Hence 
   *  - 2 resource types must be defined (named RtM1 & RtM2), one for rr_m1 and the other for rr_m2.
   *  - a resource server has to be registered on both RRs and callbacks have to be declared:
   * 
   *       * rr_m1.addServer<RtM1>(&loadCallbackRtM1, &unloadCallbackRtM1);
   *       * rr_m2.addServer<RtM2>(&loadCallbackRtM2, &unloadCallbackRtM2);
   * 
   *  - load callbacks have to be defined, where loadCallbackRtM1 makes a client call to rr_m2.
   *  - in order to test this contraption, rr_m0 must make a client call to rr_m1
   */ 

  temoto_resource_registrar::RrBase rr_m0;
  temoto_resource_registrar::RrBase rr_m1;
  temoto_resource_registrar::RrBase rr_m2;
  std::cout << "RR objects initialized" << std::endl;

  /*
   * TODO: Request a RtM1 resoure via rr_m0 for couple of times and verify that:
   *   a) the resource has been allocated once, both on rr_m1 and rr_m2
   *   b) resource reference count in rr_m1 is equal to the nr of requests by rr_m0
   */

  return 0;
}