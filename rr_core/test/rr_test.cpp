#include "glog/logging.h"
#include "gtest/gtest.h"
#include <iostream>

#include "temoto_resource_registrar/rr_base.h"
#include "temoto_resource_registrar/rr_server_base.h"

#include <unistd.h>

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

class RrBaseTest : public ::testing::Test
{
protected:
  RrBaseTest()
  {
  }

  virtual ~RrBaseTest()
  {
  }

  virtual void SetUp()
  {
  }

  virtual void TearDown()
  {
  }

  temoto_resource_registrar::RrBase rr_m0;
  temoto_resource_registrar::RrBase rr_m1;
  temoto_resource_registrar::RrBase rr_m2;
};

template <class serverClass>
class ServerDerivate : public temoto_resource_registrar::RrServerBase
{
public:
  ServerDerivate(const std::string &name, void (*loadCallback)(), void (*unLoadCallback)())
      : RrServerBase(name, __func__, loadCallback, unLoadCallback)
  {
  }
  void print()
  {
    LOG(INFO) << "I am '" << name_ << "' (" << class_name_ << "). "
              << "My resource class: "
              << typeid(serverClass).name() << " ."
              << "My ID: " << id() << std::endl;
  }
};

class SimpleResource
{
public:
  SimpleResource() : res_name_(__func__), open_(false), operation_counter_(0) {}

  SimpleResource(const std::string &className) : open_(false), operation_counter_(0)
  {
    res_name_ = className;
  }

  bool openResource()
  {
    if (!open_)
      open_ = true;
    else
      throw "Already open";
    return true;
  }

  bool closeResource()
  {
    if (open_)
      open_ = false;
    else
      throw "Already closed";
    return true;
  }

  int executeResource()
  {
    return operation_counter_++;
  }

private:
  std::string res_name_;
  bool open_;
  int operation_counter_;
};

class Resource1 : public SimpleResource
{
public:
  Resource1() : SimpleResource(__func__)
  {
  }
};

class Resource2 : public SimpleResource
{
public:
  Resource2() : SimpleResource(__func__)
  {
  }
};

void RtM1LoadCB()
{
  LOG(INFO) << "RtM1LoadCB called";
};
void RtM1UnloadCB()
{
  LOG(INFO) << "RtM1UnloadCB called";
};

void RtM2LoadCB()
{
  LOG(INFO) << "RtM2LoadCB called";
};
void RtM2UnloadCB()
{
  LOG(INFO) << "RtM2UnloadCB called";
};

TEST_F(RrBaseTest, ResourceRegistrarTest)
{

  /*
   * TODO: Request a RtM1 resoure via rr_m0 for couple of times and verify that:
   *   a) the resource has been allocated once, both on rr_m1 and rr_m2
   *   b) resource reference count in rr_m1 is equal to the nr of requests by rr_m0
   */

  LOG(INFO) << "adding resource servers to rr_m1 and rr_m2";
  rr_m1.addServer(std::make_unique<ServerDerivate<Resource1>>("Resource1 resource", &RtM1LoadCB, &RtM1UnloadCB));
  rr_m2.addServer(std::make_unique<ServerDerivate<Resource2>>("Resource2 resource", &RtM2LoadCB, &RtM2UnloadCB));
  LOG(INFO) << "adding resource done";

  LOG(INFO) << "rr_m1 servers:";
  rr_m1.print();
  LOG(INFO) << "rr_m2 servers:";
  rr_m2.print();

  //rr_m0.call();
}
