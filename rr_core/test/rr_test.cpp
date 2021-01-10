/*
#include "glog/logging.h"
#include "gtest/gtest.h"
#include <iostream>

#include "temoto_resource_registrar/rr_base.h"
#include "temoto_resource_registrar/rr_client_base.h"
#include "temoto_resource_registrar/rr_query_base.h"
#include "temoto_resource_registrar/rr_resource.h"
#include "temoto_resource_registrar/rr_server_base.h"

#include <unistd.h>
using namespace temoto_resource_registrar;
*/

//int main(int argc, char **argv)
//{
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
/*
RrBase rr_m0 = RrBase("rr_m0");
RrBase rr_m1 = RrBase("rr_m1");
RrBase rr_m2 = RrBase("rr_m2");

int loadCalls = 0;
int r1LoadCalls = 0;
int r2LoadCalls = 0;

class RrBaseTest : public ::testing::Test
{
protected:
  RrBaseTest()
  {
    //rr_m0_client = temoto_resource_registrar::RrClientBase("rr_m0_client", rr_m0 << rr_registry_);
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

  RrQueryResponse processRequest(RrQueryRequest req)
  {

    LOG(INFO) << "processing req from derivate server: " << name_;
    return RrQueryResponse("Processed data from " + name_);
  }
};

class SimpleResource : public RrResource
{
public:
  SimpleResource() : res_name_(__func__) {}

  SimpleResource(const std::string &className)
  {
    res_name_ = className;
  }

  std::string name()
  {
    return res_name_;
  }

private:
  std::string res_name_;
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
  loadCalls++;
  r1LoadCalls++;
  LOG(INFO) << "RtM1LoadCB called";

  RrQueryRequest req("request resource 2 messsage");
  RrQueryBase query = RrQueryBase("Resource2 resource", req);
  rr_m1.call(query, rr_m2);
};

static void RtM1UnloadCB()
{
  LOG(INFO) << "RtM1UnloadCB called";
};

static void RtM2LoadCB()
{
  loadCalls++;
  r2LoadCalls++;
  LOG(INFO) << "RtM2LoadCB called";
};

static void RtM2UnloadCB()
{
  LOG(INFO) << "RtM2UnloadCB called";
};

TEST_F(RrBaseTest, ResourceRegistrarTest)
{
  */

/*
   * TODO: Request a RtM1 resoure via rr_m0 for couple of times and verify that:
   *   a) the resource has been allocated once, both on rr_m1 and rr_m2
   *   b) resource reference count in rr_m1 is equal to the nr of requests by rr_m0
   */

//check that counts for servers are 0
/*
  EXPECT_EQ(rr_m0.serverCount(), 0);
  EXPECT_EQ(rr_m1.serverCount(), 0);
  EXPECT_EQ(rr_m2.serverCount(), 0);

  LOG(INFO) << "adding resource servers to rr_m1 and rr_m2";

  rr_m1.addServer(std::make_unique<ServerDerivate<Resource1>>("Resource1 resource", &RtM1LoadCB, &RtM1UnloadCB));
  rr_m2.addServer(std::make_unique<ServerDerivate<Resource2>>("Resource2 resource", &RtM2LoadCB, &RtM2UnloadCB));
  LOG(INFO) << "adding resource done";

  // check that servers registered correctly
  EXPECT_EQ(rr_m0.serverCount(), 0);
  EXPECT_EQ(rr_m1.serverCount(), 1);
  EXPECT_EQ(rr_m2.serverCount(), 1);

  LOG(INFO) << "rr_m0 servers:";
  rr_m0.print();
  LOG(INFO) << "rr_m1 servers:";
  rr_m1.print();
  LOG(INFO) << "rr_m2 servers:";
  rr_m2.print();

  // check call counters are 0
  EXPECT_EQ(loadCalls, 0);
  EXPECT_EQ(r1LoadCalls, 0);
  EXPECT_EQ(r2LoadCalls, 0);

  LOG(INFO) << "executing call to rr_m0";
  RrQueryRequest req("request resource 1 messsage");
  RrQueryBase query("Resource1 resource", req);

  EXPECT_EQ(query.response().response_, "");

  LOG(INFO) << "calling...";
  rr_m0.call(query, rr_m1);

  EXPECT_EQ(loadCalls, 2);
  EXPECT_EQ(r1LoadCalls, 1);
  EXPECT_EQ(r2LoadCalls, 1);

  // check that message is expected
  EXPECT_EQ(query.response().response_, "Processed data from Resource1 resource");

  // execute query again and check that reload is not done
  LOG(INFO)
      << "executing call to rr_m0";
  LOG(INFO) << "calling...";

  req = RrQueryRequest("request resource 1 messsage");
  query = RrQueryBase("Resource1 resource", req);

  EXPECT_EQ(query.response().response_, "");

  rr_m0.call(query, rr_m1);

  EXPECT_EQ(loadCalls, 2);
  EXPECT_EQ(r1LoadCalls, 1);
  EXPECT_EQ(r2LoadCalls, 1);

  // check that message is expected
  EXPECT_EQ(query.response().response_, "Processed data from Resource1 resource");
}
*/