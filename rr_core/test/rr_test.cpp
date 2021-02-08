#include "glog/logging.h"
#include "gtest/gtest.h"

#include <iostream>
#include <sstream>
#include <thread>
#include <unistd.h>

#include "temoto_resource_registrar/rr_base.h"
#include "temoto_resource_registrar/rr_client_base.h"
#include "temoto_resource_registrar/rr_resource.h"
#include "temoto_resource_registrar/rr_serializable.h"
#include "temoto_resource_registrar/rr_server_base.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

using namespace temoto_resource_registrar;

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

RrBase<RrServerBase, RrClientBase> rr_m0("rr_m0");
RrBase<RrServerBase, RrClientBase> rr_m1("rr_m1");
RrBase<RrServerBase, RrClientBase> rr_m2("rr_m2");

std ::string expectedMessage = "";

int loadCalls = 0;
int r1LoadCalls = 0;
int r2LoadCalls = 0;

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
};

template <class MessageType>
class RrQueryRequestTemplate : public RrQueryRequest
{
public:
  RrQueryRequestTemplate() : requestId_(boost::uuids::to_string(boost::uuids::random_generator()()))
  {
    LOG(INFO) << "generated id: " << requestId_;
  }

  RrQueryRequestTemplate(const MessageType &request) : message_(request)
  {
    RrQueryRequestTemplate();
  };

  MessageType getRequest() const
  {
    return message_;
  };

  std::string getId()
  {
    return requestId_;
  }

protected:
  std::string requestId_;

  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /* version */)
  {
    ar &requestId_ &message_;
  }

private:
  MessageType message_;
};

template <class MessageType>
class RrQueryResponseTemplate : public RrQueryResponse
{
public:
  RrQueryResponseTemplate() {}

  RrQueryResponseTemplate(const MessageType &response) : response_(response){};

  MessageType getResponse() const
  {
    return response_;
  };

protected:
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /* version */)
  {
    ar &response_;
  }

private:
  MessageType response_;
};

template <class MessageType>
class RrQueryTemplate : public RrQueryBase
{
public:
  RrQueryTemplate() {}

  RrQueryTemplate(const RrQueryRequestTemplate<MessageType> &request, const RrQueryResponseTemplate<MessageType> &response) : typed_request_(request), typed_response_(response){};

  RrQueryRequestTemplate<MessageType> request() const
  {
    return typed_request_;
  }

  void storeResponse(RrQueryResponseTemplate<MessageType> response)
  {
    typed_response_ = response;
  }

  RrQueryResponseTemplate<MessageType> response() const
  {
    return typed_response_;
  }

protected:
  RrQueryRequestTemplate<MessageType> typed_request_;
  RrQueryResponseTemplate<MessageType> typed_response_;

  std::string requestId_;

  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /* version */)
  {
    ar &requestId_ &typed_request_ &typed_response_;
  }
};

template <class MessageType>
class RrClientTemplate : public RrClientBase, Serializable<MessageType>
{
public:
  RrClientTemplate(const std::string &name) : RrClientBase(name){};

  void invoke(const RrQueryBase &query) const
  {
    LOG(INFO) << "invoke done - override";
  }

  std::string serialize(MessageType message) const {};
  MessageType deserialize(const std::string &data) const {};

protected:
private:
};

template <class MessageType>
class RrTemplateServer : public RrServerBase
{

public:
  RrTemplateServer(const std::string &name,
                   void (*loadCallback)(RrQueryTemplate<MessageType> &),
                   void (*unLoadCallback)(RrQueryTemplate<MessageType> &)) : RrServerBase(name,
                                                                                          NULL,
                                                                                          NULL),
                                                                             typed_load_callback_ptr_(loadCallback),
                                                                             typed_unload_callback_ptr_(unLoadCallback){};

  void processQuery(RrQueryTemplate<MessageType> &query) const
  {
    LOG(INFO) << "processQuery - override - " << typeid(MessageType).name() << "server: " << name_;
    // need to call server here.

    MessageType rawRequest = query.request().getRequest();
    std::string serializedRequest = serialize<MessageType>(rawRequest);

    LOG(INFO) << "checking existance of: " << name_ << " - " << serializedRequest;

    if (!rr_catalog_->hasStoredServerQuery(name_, serializedRequest))
    {
      LOG(INFO) << "Request not found. Running and storing it";
      typed_load_callback_ptr_(query);
      LOG(INFO) << "Finished callback";

      LOG(INFO) << "Storing query data..." << name_ << " - " << serializedRequest;
      rr_catalog_->storeServerQuery(name_, query.request().getId(), serializedRequest, serialize<RrQueryTemplate<MessageType>>(query));
      LOG(INFO) << "Finished storing";
    }
    else
    {
      LOG(INFO) << "Request found. No storage needed. Fetching it... ";
      RrQueryTemplate<MessageType> storedQuery = deserialize<RrQueryTemplate<MessageType>>(rr_catalog_->fetchFromServerStorage(name_, serializedRequest));
      LOG(INFO) << "Fetching done";

      query.storeResponse(storedQuery.response());
    }
  };

  bool unloadMessage(const std::string &id)
  {
    LOG(INFO) << "unloading resource: " << id;

    return rr_catalog_->removeServerQuery(name_, id);
  }

  template <class SerialClass>
  std::string serialize(SerialClass message) const
  {

    std::stringstream ss;
    boost::archive::binary_oarchive oa(ss);

    oa << message;

    return ss.str();
  };

  template <class SerialClass>
  SerialClass deserialize(const std::string &data) const
  {
    std::stringstream ss(data);
    boost::archive::binary_iarchive ia(ss);

    SerialClass obj;
    ia >> obj;

    return obj;
  };

protected:
  void (*typed_load_callback_ptr_)(RrQueryTemplate<MessageType> &);
  void (*typed_unload_callback_ptr_)(RrQueryTemplate<MessageType> &);

private:
};

class Resource1
{
public:
  Resource1()
  {
  }

  Resource1(const std::string &message) : message_(message)
  {
  }

  std::string rawMessage()
  {
    return message_;
  }

protected:
  std::string message_;

  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /* version */)
  {
    ar &message_;
  }
};

class Resource2
{
public:
  Resource2()
  {
  }

  Resource2(int i, int j) : i_(i), j_(j)
  {
  }

  int i_;
  int j_;

protected:
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /* version */)
  {
    ar &i_ &j_;
  }
};

void RtM1LoadCB(RrQueryTemplate<Resource1> &query)
{
  loadCalls++;
  r1LoadCalls++;
  LOG(INFO) << "RtM1LoadCB called";

  EXPECT_EQ(query.request().getRequest().rawMessage(), expectedMessage);

  RrQueryRequestTemplate<Resource2> req(Resource2(1, 0));
  RrQueryResponseTemplate<Resource2> resp(Resource2(0, 0));
  RrQueryTemplate<Resource2> newQuery(req, resp);

  rr_m1.call<RrTemplateServer<Resource2>, RrQueryTemplate<Resource2>>(rr_m2, "R2_S", newQuery);

  EXPECT_EQ(newQuery.response().getResponse().j_, 100);
  EXPECT_EQ(newQuery.response().getResponse().i_, 1);

  query.storeResponse(RrQueryResponseTemplate<Resource1>(Resource1("Everything Works")));
};

void RtM1UnloadCB(RrQueryTemplate<Resource1> &query)
{
  LOG(INFO) << "RtM1UnloadCB called";
};

void RtM2LoadCB(RrQueryTemplate<Resource2> &query)
{
  loadCalls++;
  r2LoadCalls++;
  LOG(INFO) << "RtM2LoadCB called";

  LOG(INFO) << "Setting response";

  Resource2 rawMessage = query.request().getRequest();

  EXPECT_EQ(rawMessage.i_, 1);
  EXPECT_EQ(rawMessage.j_, 0);

  query.storeResponse(RrQueryResponseTemplate<Resource2>(Resource2(rawMessage.i_, 100)));
  LOG(INFO) << "Done Setting response";
};

void RtM2UnloadCB(RrQueryTemplate<Resource2> &query)
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

  //check that counts for servers are 0

  std::vector<std::string> ids;

  EXPECT_EQ(rr_m0.serverCount(), 0);
  EXPECT_EQ(rr_m1.serverCount(), 0);
  EXPECT_EQ(rr_m2.serverCount(), 0);

  LOG(INFO) << "adding resource servers to rr_m1 and rr_m2";

  LOG(INFO) << std::this_thread::get_id();

  rr_m1.registerServer(std::make_unique<RrTemplateServer<Resource1>>("R1_S", &RtM1LoadCB, &RtM1UnloadCB));
  rr_m2.registerServer(std::make_unique<RrTemplateServer<Resource2>>("R2_S", &RtM2LoadCB, &RtM2UnloadCB));
  LOG(INFO) << "adding resource done";

  // check that servers registered correctly
  EXPECT_EQ(rr_m0.serverCount(), 0);
  EXPECT_EQ(rr_m1.serverCount(), 1);
  EXPECT_EQ(rr_m2.serverCount(), 1);

  // check call counters are 0
  EXPECT_EQ(loadCalls, 0);
  EXPECT_EQ(r1LoadCalls, 0);
  EXPECT_EQ(r2LoadCalls, 0);

  LOG(INFO) << "executing call to rr_m0";
  expectedMessage = "testMessage here";
  RrQueryTemplate<Resource1> query(Resource1("testMessage here"), Resource1(""));

  ids.push_back(query.request().getId());

  LOG(INFO) << "calling...";
  //rr_m0.call<RrClientTemplate<Resource1>>("rr_m1", "R1_S", query);
  rr_m0.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_m1, "R1_S", query);

  EXPECT_EQ(loadCalls, 2);
  EXPECT_EQ(r1LoadCalls, 1);
  EXPECT_EQ(r2LoadCalls, 1);

  EXPECT_EQ(query.response().getResponse().rawMessage(), "Everything Works");

  LOG(INFO) << "-------------------";

  rr_m0.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_m1, "R1_S", query);

  EXPECT_EQ(loadCalls, 2);
  EXPECT_EQ(r1LoadCalls, 1);
  EXPECT_EQ(r2LoadCalls, 1);

  LOG(INFO) << "-------------------";

  LOG(INFO) << "executing new call to rr_m0";

  expectedMessage = "newMessage here";
  RrQueryRequestTemplate<Resource1> newReq(Resource1("newMessage here"));
  RrQueryResponseTemplate<Resource1> newResp(Resource1(""));
  RrQueryTemplate<Resource1> newQuery(newReq, newResp);

  ids.push_back(newReq.getId());

  rr_m0.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_m1, "R1_S", newQuery);

  EXPECT_EQ(query.response().getResponse().rawMessage(), "Everything Works");

  EXPECT_EQ(loadCalls, 3);
  EXPECT_EQ(r1LoadCalls, 2);
  EXPECT_EQ(r2LoadCalls, 1);

  LOG(INFO) << "-------------------";
  LOG(INFO) << "-------------------";
  LOG(INFO) << "-------------------";

  LOG(INFO) << "Unloading results!";

  LOG(INFO) << "ids size " << ids.size();

  for (auto const &i : ids)
  {
    LOG(INFO) << "possible ID: " << i;
    LOG(INFO) << rr_m1.unload<RrTemplateServer<Resource1>>("R1_S", i);
  }
}
