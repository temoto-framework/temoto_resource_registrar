#include "gtest/gtest.h"
#include <iostream>

#include "temoto_resource_registrar/rr_base.h"

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

TEST_F(RrBaseTest, RRinitProcess)
{
  rr_m0.print();
}