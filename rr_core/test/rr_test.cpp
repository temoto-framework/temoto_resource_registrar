#include "glog/logging.h"
#include "gtest/gtest.h"

#include <iostream>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <unordered_map>

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

RrBase<RrServerBase, RrClientBase> rr_m0("rr_m0");
RrBase<RrServerBase, RrClientBase> rr_m1("rr_m1");
RrBase<RrServerBase, RrClientBase> rr_m1_1("rr_m1_1");
RrBase<RrServerBase, RrClientBase> rr_m2("rr_m2");

std::unordered_map<std::string, RrBase<RrServerBase, RrClientBase> *> rr_references_;

std::string expectedMessage = "";

int loadCalls = 0;
int r1LoadCalls = 0;
int r2LoadCalls = 0;
int r1UnLoadCalls = 0;
int r2UnLoadCalls = 0;

int statusCalls = 0;

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
  RrQueryRequestTemplate() {}

  RrQueryRequestTemplate(const MessageType &request) : message_(request)
  {
  }

  MessageType getRequest() const
  {
    return message_;
  }

protected:
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /* version */)
  {
    ar &message_;
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

  friend class boost::serialization::access;

  std::string requestId_;

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

    query.setId(generateId());

    LOG(INFO) << "checking existance of: " << name_ << " - " << query.id();
    std::string requestId = rr_catalog_->queryExists(name_, serializedRequest);
    if (requestId.size() == 0)
    {
      LOG(INFO) << "Request not found. Running and storing it";
      typed_load_callback_ptr_(query);
      LOG(INFO) << "Finished callback";

      LOG(INFO) << "Storing query data to server..." << name_;

      storeQuery(serializedRequest, query);
      LOG(INFO) << "Finished storing";

      if (query.dependencies().size())
      {
        LOG(INFO) << "dependency storage required";
        for (auto const &i : query.dependencies())
        {
          LOG(INFO) << "     dependency: " << i.first << " - " << i.second;

          rr_catalog_->storeDependency(query.id(), i.second, i.first);
        }
      }
    }
    else
    {
      LOG(INFO) << "Request found. No storage needed. Fetching it... ";
      std::string serializedQuery = processExisting(requestId, query);
      RrQueryTemplate<MessageType> previousRequest = deserialize<RrQueryTemplate<MessageType>>(serializedQuery);

      query.storeResponse(previousRequest.response());
      LOG(INFO) << "Fetching done... " << serializedQuery;
    }
  };

  bool unloadMessage(const std::string &id)
  {
    LOG(INFO) << "unloading resource: " << id;

    LOG(INFO) << "checkign for dependencies... ";

    std::string query = rr_catalog_->unload(name_, id);

    RrQueryTemplate<MessageType> q = deserialize<RrQueryTemplate<MessageType>>(query);

    LOG(INFO) << "Unload result size: " << query.size();

    if (rr_catalog_->canBeUnloaded(name_))
    {
      RrQueryTemplate<MessageType> q = deserialize<RrQueryTemplate<MessageType>>(query);
      LOG(INFO) << "Time for unload CB!";
      typed_unload_callback_ptr_(q);
    }

    return query.size() > 0;
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

  std::string generateId() const
  {
    return boost::uuids::to_string(boost::uuids::random_generator()());
  }

private:
  void storeQuery(const std::string &rawRequest, RrQueryTemplate<MessageType> query) const
  {
    rr_catalog_->storeQuery(name_,
                            query,
                            rawRequest,
                            serialize<RrQueryTemplate<MessageType>>(query));
  }

  std::string processExisting(const std::string &requestId, RrQueryTemplate<MessageType> query) const
  {
    return rr_catalog_->processExisting(name_, requestId, query);
  };
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

  rr_m1.call<RrTemplateServer<Resource2>, RrQueryTemplate<Resource2>>(rr_m2, "R2_S", newQuery, &(query));

  EXPECT_EQ(newQuery.response().getResponse().j_, 100);
  EXPECT_EQ(newQuery.response().getResponse().i_, 1);

  RrQueryRequestTemplate<Resource2> req2(Resource2(2, 0));
  RrQueryResponseTemplate<Resource2> resp2(Resource2(0, 0));
  RrQueryTemplate<Resource2> newQuery2(req2, resp2);

  rr_m1.call<RrTemplateServer<Resource2>, RrQueryTemplate<Resource2>>(rr_m2, "R2_S", newQuery2, &(query));

  EXPECT_EQ(newQuery2.response().getResponse().j_, 100);
  EXPECT_EQ(newQuery2.response().getResponse().i_, 2);

  query.storeResponse(RrQueryResponseTemplate<Resource1>(Resource1("Everything Works")));
};

void RtM1UnloadCB(RrQueryTemplate<Resource1> &query)
{
  r1UnLoadCalls++;
  LOG(INFO) << "RtM1UnloadCB called";
};

void RtM2LoadCB(RrQueryTemplate<Resource2> &query)
{
  loadCalls++;
  r2LoadCalls++;
  LOG(INFO) << "RtM2LoadCB called";

  LOG(INFO) << "Setting response";

  Resource2 rawMessage = query.request().getRequest();

  //EXPECT_EQ(rawMessage.i_, 1);
  //EXPECT_EQ(rawMessage.j_, 0);

  query.storeResponse(RrQueryResponseTemplate<Resource2>(Resource2(rawMessage.i_, 100)));

  LOG(INFO) << "Done Setting response";
};

void RtM2UnloadCB(RrQueryTemplate<Resource2> &query)
{
  r2UnLoadCalls++;
  LOG(INFO) << "RtM2UnloadCB called";
};

void RRStatusFucntion(const std::string &id, Status status, std::string &message)
{
  LOG(INFO) << "RRStatusFucntion called " << id;
  EXPECT_EQ(status, Status::UPDATE);
  EXPECT_EQ(message, "All OK");

  statusCalls++;
};

TEST_F(RrBaseTest, ResourceRegistrarTest)
{

  /*
   * TODO: Request a RtM1 resoure via rr_m0 for couple of times and verify that:
   *   a) the resource has been allocated once, both on rr_m1 and rr_m2
   *   b) resource reference count in rr_m1 is equal to the nr of requests by rr_m0
   */

  rr_references_["rr_m0"] = &rr_m0;
  rr_references_["rr_m1"] = &rr_m1;
  rr_references_["rr_m1_1"] = &rr_m1_1;
  rr_references_["rr_m2"] = &rr_m2;
  rr_m0.setRrReferences(rr_references_);
  rr_m1.setRrReferences(rr_references_);
  rr_m1_1.setRrReferences(rr_references_);
  rr_m2.setRrReferences(rr_references_);

  //check that counts for servers are 0

  std::vector<std::string> ids;

  EXPECT_EQ(rr_m0.callbacks().size(), 0);
  EXPECT_EQ(rr_m1.callbacks().size(), 0);
  EXPECT_EQ(rr_m1_1.callbacks().size(), 0);
  EXPECT_EQ(rr_m2.callbacks().size(), 0);

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

  LOG(INFO) << "calling...";

  LOG(INFO) << "query : -------------------------------------------------------";

  //rr_m0.call<RrClientTemplate<Resource1>>("rr_m1", "R1_S", query);
  rr_m0.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_m1, "R1_S", query, NULL, RRStatusFucntion);

  EXPECT_EQ(loadCalls, 3);
  EXPECT_EQ(r1LoadCalls, 1);
  EXPECT_EQ(r2LoadCalls, 2);

  EXPECT_EQ(query.response().getResponse().rawMessage(), "Everything Works");

  LOG(INFO) << "registered callback for query with id: " << query.id();
  EXPECT_EQ(rr_m0.callbacks().size(), 1);
  EXPECT_EQ(rr_m0.callbacks().count(query.id()), 1);

  ids.push_back(query.id());

  LOG(INFO) << "requery : -------------------------------------------------------";

  //rr_m0.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_m1, "R1_S", query);

  EXPECT_EQ(loadCalls, 3);
  EXPECT_EQ(r1LoadCalls, 1);
  EXPECT_EQ(r2LoadCalls, 2);

  //ids.push_back(query.id());

  LOG(INFO) << "new query : -------------------------------------------------------";

  LOG(INFO) << "executing new call to rr_m0";

  expectedMessage = "newMessage here";
  RrQueryRequestTemplate<Resource1> newReq(Resource1("newMessage here"));
  RrQueryResponseTemplate<Resource1> newResp(Resource1(""));
  RrQueryTemplate<Resource1> newQuery(newReq, newResp);

  rr_m0.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_m1, "R1_S", newQuery);

  ids.push_back(newQuery.id());

  EXPECT_EQ(query.response().getResponse().rawMessage(), "Everything Works");

  EXPECT_EQ(loadCalls, 4);
  EXPECT_EQ(r1LoadCalls, 2);
  EXPECT_EQ(r2LoadCalls, 2);

  LOG(INFO) << "existing query from rr_m1_1 to m2 -------------------------------------------------------";
  RrQueryRequestTemplate<Resource2> req2(Resource2(1, 0));
  RrQueryResponseTemplate<Resource2> resp2(Resource2(0, 0));
  RrQueryTemplate<Resource2> newQuery2(req2, resp2);

  rr_m1_1.call<RrTemplateServer<Resource2>, RrQueryTemplate<Resource2>>(rr_m2, "R2_S", newQuery2);
  std::string idForStatus = newQuery2.id();

  LOG(INFO) << "existing query from rr_m1_1 to m2 -------------------------------------------------------";

  LOG(INFO) << "RR_M0 Catalog -------------------";
  rr_m0.printCatalog();
  LOG(INFO) << "RR_M0 Catalog -------------------";

  LOG(INFO) << "RR_M1 Catalog -------------------";
  rr_m1.printCatalog();
  LOG(INFO) << "RR_M1 Catalog -------------------";

  LOG(INFO) << "RR_M2 Catalog -------------------";
  rr_m2.printCatalog();
  LOG(INFO) << "RR_M2 Catalog -------------------";

  LOG(INFO) << "-------------------";
  LOG(INFO) << "------------------- STATUS TESTS";

  EXPECT_EQ(statusCalls, 0);

  std::string updateMessage = "All OK";
  rr_m2.sendStatus(idForStatus, Status::UPDATE, updateMessage);

  rr_m1_1.unload(rr_m2, idForStatus);

  EXPECT_EQ(statusCalls, 1);

  LOG(INFO) << "status test for id end: " << idForStatus;
  LOG(INFO) << "--------------------------------------------------";
  LOG(INFO) << "Unloading results!";

  LOG(INFO) << "ids to unload: " << ids.size();

  for (std::string const &i : ids)
  {
    LOG(INFO) << "possible to unload ID: " << i;
    LOG(INFO) << rr_m0.unload(rr_m1, i);
    LOG(INFO) << "-------------------";
  }

  EXPECT_EQ(rr_m0.callbacks().size(), 0);
  EXPECT_EQ(rr_m1.callbacks().size(), 0);
  EXPECT_EQ(rr_m1_1.callbacks().size(), 0);
  EXPECT_EQ(rr_m2.callbacks().size(), 0);

  EXPECT_EQ(r1UnLoadCalls, 1);
  EXPECT_EQ(r2UnLoadCalls, 1);

  LOG(INFO) << "RR_M0 Catalog -------------------";
  rr_m0.printCatalog();
  LOG(INFO) << "RR_M0 Catalog -------------------";

  LOG(INFO) << "RR_M1 Catalog -------------------";
  rr_m1.printCatalog();
  LOG(INFO) << "RR_M1 Catalog -------------------";

  LOG(INFO) << "RR_M2 Catalog -------------------";
  rr_m2.printCatalog();
  LOG(INFO) << "RR_M2 Catalog -------------------";

  LOG(INFO) << "FINAL QUERY -------------------";

  expectedMessage = "testMessage here";
  query = RrQueryTemplate<Resource1>(Resource1("testMessage here"), Resource1(""));

  rr_m0.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_m1, "R1_S", query);

  EXPECT_EQ(loadCalls, 7);
  EXPECT_EQ(r1LoadCalls, 3);
  EXPECT_EQ(r2LoadCalls, 4);
}
