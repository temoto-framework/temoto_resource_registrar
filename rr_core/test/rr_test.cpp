#include "glog/logging.h"
#include "gtest/gtest.h"
#include <iostream>

#include "temoto_resource_registrar/rr_base.h"
#include "temoto_resource_registrar/rr_server_base.h"

#include <unistd.h>

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
};

class RtM1
{
};

class RtM2
{
};

void RtM1LoadCB(){};
void RtM1UnloadCB(){};

void RtM2LoadCB(){};
void RtM2UnloadCB(){};

TEST_F(RrBaseTest, ResourceRegistrarTest)
{

  auto resource1 = std::make_unique<ServerDerivate<RtM1>>("RtM1 resource", &RtM1LoadCB, &RtM1UnloadCB);
  auto resource2 = std::make_unique<ServerDerivate<RtM2>>("RtM2 resource", &RtM2LoadCB, &RtM2UnloadCB);
  rr_m0.print();
}
