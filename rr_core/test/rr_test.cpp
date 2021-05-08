#include "glog/logging.h"
#include "gtest/gtest.h"

#include "console_bridge/console.h"

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>

#include "temoto_resource_registrar/rr_base.h"
#include "temoto_resource_registrar/rr_configuration.h"
#include "temoto_resource_registrar/rr_serializable.h"
#include "temoto_resource_registrar/rr_serializer.h"
#include "temoto_resource_registrar/rr_server_base.h"
#include "temoto_resource_registrar/temoto_error.h"

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

RrBase rr_m0("rr_m0");
RrBase rr_m1("rr_m1");
RrBase rr_m1_1("rr_m1_1");
RrBase rr_m2("rr_m2");

std::unordered_map<std::string, RrBase *> rr_references_;

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
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
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

  std::string request_id_;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /* version */)
  {
    ar &request_id_ &typed_request_ &typed_response_;
  }
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

  RrTemplateServer(const std::string &name,
                   std::function<void(RrQueryTemplate<MessageType> &)> load,
                   std::function<void(RrQueryTemplate<MessageType> &)> unload,
                   bool lambdas) : RrServerBase(name,
                                                NULL,
                                                NULL),
                                   typed_load_callback_ptr_(NULL),
                                   typed_unload_callback_ptr_(NULL),
                                   typed_load_fn_(load),
                                   typed_unload_fn_(unload)
  {
  }

  RrTemplateServer(const std::string &name,
                   std::function<void(RrQueryTemplate<MessageType> &)> load,
                   std::function<void(RrQueryTemplate<MessageType> &)> unload,
                   std::function<void(MessageType, const Status &)> status) : RrServerBase(name,
                                                                                           NULL,
                                                                                           NULL),
                                                                              typed_load_callback_ptr_(NULL),
                                                                              typed_unload_callback_ptr_(NULL),
                                                                              typed_load_fn_(load),
                                                                              typed_unload_fn_(unload),
                                                                              typed_status_fn_(status)
  {
  }

  void processQuery(RrQueryTemplate<MessageType> &query) const
  {
    LOG(INFO) << "processQuery - override - " << typeid(MessageType).name() << "server: " << id_;
    // need to call server here.

    MessageType rawRequest = query.request().getRequest();
    std::string serializedRequest = Serializer::serialize<MessageType>(rawRequest);

    query.setId(generateId());

    LOG(INFO) << "checking existance of: " << id_ << " - " << query.id();
    std::string requestId = rr_catalog_->queryExists(id_, serializedRequest);
    if (requestId.size() == 0)
    {
      try
      {
        LOG(INFO) << "Executing query startup callback";
        transaction_callback_ptr_(TransactionInfo(100, query));

        LOG(INFO) << "Request not found. Running and storing it";
        if (typed_load_callback_ptr_ != NULL)
          typed_load_callback_ptr_(query);
        else
          typed_load_fn_(query);

        LOG(INFO) << "Finished callback";

        LOG(INFO) << "Storing query data to server..." << id_;

        storeQuery(serializedRequest, query);
        LOG(INFO) << "Finished storing";
      }
      catch (const resource_registrar::TemotoErrorStack &e)
      {
        LOG(INFO) << "server caught a callback exception. returning error to requestor.";
        query.metadata().errorStack().appendError(e);
      }

      LOG(INFO) << "Executing query finished callback";
      transaction_callback_ptr_(TransactionInfo(200, query));
    }
    else
    {
      LOG(INFO) << "Request found. No storage needed. Fetching it... ";
      std::string serializedQuery = processExisting(requestId, query);
      RrQueryTemplate<MessageType> previousRequest = Serializer::deserialize<RrQueryTemplate<MessageType>>(serializedQuery);

      query.storeResponse(previousRequest.response());
      LOG(INFO) << "Fetching done... " << serializedQuery;
    }
  };

  bool unloadMessage(const std::string &id)
  {
    LOG(INFO) << "unloading resource: " << id;

    LOG(INFO) << "checkign for dependencies... ";

    bool canUnload = false;

    std::string query = rr_catalog_->unload(id_, id, canUnload);

    RrQueryTemplate<MessageType> q = Serializer::deserialize<RrQueryTemplate<MessageType>>(query);

    LOG(INFO) << "Unload result size: " << query.size();

    if (canUnload)
    {
      RrQueryTemplate<MessageType> q = Serializer::deserialize<RrQueryTemplate<MessageType>>(query);
      LOG(INFO) << "Time for unload CB!";
      if (typed_unload_callback_ptr_ != NULL)
        typed_unload_callback_ptr_(q);
      else
        typed_unload_fn_(q);
    }

    return query.size() > 0;
  }

  void triggerCallback(const Status &status) const
  {
    LOG(INFO) << "triggerCallback triggered for server " << id();

    if (typed_status_fn_ != NULL)
    {
      MessageType q = Serializer::deserialize<MessageType>(status.serialised_request_);
      typed_status_fn_(q, status);
    }
  }

protected:
  void (*typed_load_callback_ptr_)(RrQueryTemplate<MessageType> &);
  void (*typed_unload_callback_ptr_)(RrQueryTemplate<MessageType> &);

  std::function<void(RrQueryTemplate<MessageType> &)> typed_load_fn_;
  std::function<void(RrQueryTemplate<MessageType> &)> typed_unload_fn_;
  std::function<void(MessageType, const Status &)> typed_status_fn_;

private:
  void storeQuery(const std::string &rawRequest, RrQueryTemplate<MessageType> query) const
  {
    rr_catalog_->storeQuery(id_,
                            query,
                            rawRequest,
                            Serializer::serialize<RrQueryTemplate<MessageType>>(query));
  }

  std::string processExisting(const std::string &requestId, RrQueryTemplate<MessageType> query) const
  {
    return rr_catalog_->processExisting(id_, requestId, query);
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

  rr_m1.call<RrTemplateServer<Resource2>, RrQueryTemplate<Resource2>>(rr_m2, "R2_S", newQuery);

  EXPECT_EQ(newQuery.response().getResponse().j_, 100);
  EXPECT_EQ(newQuery.response().getResponse().i_, 1);

  RrQueryRequestTemplate<Resource2> req2(Resource2(2, 0));
  RrQueryResponseTemplate<Resource2> resp2(Resource2(0, 0));
  RrQueryTemplate<Resource2> newQuery2(req2, resp2);

  rr_m1.call<RrTemplateServer<Resource2>, RrQueryTemplate<Resource2>>(rr_m2, "R2_S", newQuery2);

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

template <typename T>
bool contains(std::vector<T> vec, const T &elem)
{
  bool result = false;
  if (find(vec.begin(), vec.end(), elem) != vec.end())
  {
    result = true;
  }
  return result;
}

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
  //rr_m0.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_m1, "R1_S", query, NULL, RRStatusFucntion);
  rr_m0.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_m1, "R1_S", query);

  EXPECT_EQ(loadCalls, 3);
  EXPECT_EQ(r1LoadCalls, 1);
  EXPECT_EQ(r2LoadCalls, 2);

  EXPECT_EQ(query.response().getResponse().rawMessage(), "Everything Works");

  LOG(INFO) << "registered callback for query with id: " << query.id();
  //EXPECT_EQ(rr_m0.callbacks().size(), 1);
  //EXPECT_EQ(contains(rr_m0.callbacks(), query.id()), true);

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

  //EXPECT_EQ(statusCalls, 0);

  //std::string updateMessage = "All OK";
  //rr_m2.sendStatus(idForStatus, Status::UPDATE, updateMessage);

  rr_m1_1.unload(rr_m2, idForStatus);

  //EXPECT_EQ(statusCalls, 1);

  LOG(INFO) << "--------------------------------------------------";
  //rr_m1.getServerQueries<Resource1>("R1_S");
  LOG(INFO) << "--------------------------------------------------";

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

  EXPECT_EQ(r1UnLoadCalls, 2);
  EXPECT_EQ(r2UnLoadCalls, 2);

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

TEST_F(RrBaseTest, RegistrarConfigurationTest)
{

  {
    Configuration config;
    config.setName("testName")
        ->setLocation("./catalogBackup.backup")
        ->setSaveInterval(100)
        ->setSaveOnModify(true)
        ->setEraseOnDestruct(true);

    EXPECT_EQ(config.name(), "testName");
    EXPECT_EQ(config.location(), "./catalogBackup.backup");
    EXPECT_EQ(config.saveInterval(), 100);
    EXPECT_TRUE(config.saveOnModify());

    RrCatalog catalog;
    RrQueryBase query;
    query.setId("queryId1");
    query.setOrigin("originRR");
    query.setRr("RR");

    catalog.storeQuery("server1", query, "reqData", "qData");
    catalog.storeClientCallRecord("clientName", "queryId2");
    catalog.storeDependency("queryId1", "rr2", "dependencyId1");

    RrBase rr(config);
    EXPECT_EQ(rr.name(), "testName");

    rr.updateCatalog(catalog);

    rr.saveCatalog();
    rr.loadCatalog();

    rr.saveCatalog();
    rr.loadCatalog();

    config.setLocation("./catalogBackup.backup2");
    rr.updateConfiguration(config);

    rr.saveCatalog();
    rr.loadCatalog();

    rr.saveCatalog();
    rr.loadCatalog();

    std::ifstream s1("./catalogBackup.backup");
    std::ifstream s2("./catalogBackup.backup2");

    std::string str1;
    std::string str2;

    s1.seekg(0, std::ios::end);
    str1.reserve(s1.tellg());
    s1.seekg(0, std::ios::beg);
    str1.assign((std::istreambuf_iterator<char>(s1)),
                std::istreambuf_iterator<char>());

    s2.seekg(0, std::ios::end);
    str2.reserve(s2.tellg());
    s2.seekg(0, std::ios::beg);
    str2.assign((std::istreambuf_iterator<char>(s2)),
                std::istreambuf_iterator<char>());

    LOG(INFO) << "comparing results";
    EXPECT_EQ(str1.size(), str2.size());

    LOG(INFO) << "loading backup contents...";
    std::ifstream ifs("./catalogBackup.backup", std::ios::binary);
    boost::archive::binary_iarchive ia(ifs);
    RrCatalog loadedCatalog;
    ia >> loadedCatalog;

    LOG(INFO) << "checking backup catalog contents...";
    QueryContainer<std::string> cont = loadedCatalog.findOriginalContainer("queryId1");
    EXPECT_EQ(cont.raw_query_, "qData");
    EXPECT_EQ(cont.raw_request_, "reqData");
    EXPECT_EQ(cont.responsible_server_, "server1");
    EXPECT_EQ(cont.getIdCount(), 1);

    EXPECT_EQ(cont.q_.id(), query.id());
    EXPECT_EQ(cont.q_.origin(), query.origin());
    EXPECT_EQ(cont.q_.rr(), query.rr());

    std::unordered_map<UUID, std::string> dep = loadedCatalog.getDependencies("queryId1");
    EXPECT_EQ(dep.size(), 1);
    EXPECT_EQ(dep.count("dependencyId1"), 1);
    EXPECT_EQ(dep["dependencyId1"], "rr2");

    std::string clientId = loadedCatalog.getIdClient("queryId2");
    EXPECT_EQ(clientId, "clientName");
  }
  LOG(INFO) << "Destroy delete test";

  if (remove("./catalogBackup.backup") != 0)
  {
    FAIL() << "File ./catalogBackup.backup deletion failed.";
  }
  if (remove("./catalogBackup.backup2") == 0)
  {
    FAIL() << "File ./catalogBackup.backup2 was not deleted by destructor.";
  }
}

TEST_F(RrBaseTest, CatalogResponseUpdateTest)
{
  RrCatalog catalog;

  RrQueryBase query;
  query.setId("queryId1");
  query.setOrigin("originRR");
  query.setRr("RR");

  catalog.storeQuery("server", query, "request", "");

  QueryContainer<std::string> orig = catalog.findOriginalContainer("queryId1");

  EXPECT_EQ(orig.raw_query_, "");

  catalog.updateResponse("server", "request", "updatedResponse");

  EXPECT_EQ(catalog.queryExists("server", "request"), "queryId1");

  QueryContainer<std::string> container = catalog.findOriginalContainer("queryId1");

  EXPECT_EQ(container.raw_query_, "updatedResponse");
}

TEST_F(RrBaseTest, ClientUnloadTest)
{

  std::map<std::string, int> unloadCounter;

  class UnloadTestRr : public RrBase
  {
  public:
    UnloadTestRr(const std::string &name, std::map<std::string, int> &map) : RrBase(name), map(map) {}

    ~UnloadTestRr()
    {
      LOG(INFO) << "unloading clients";
      for (const std::string &clientId : clients_.getIds())
      {
        try
        {
          LOG(INFO) << "unloading client " << clientId;
          unloadClient(clientId);
        }
        catch (...)
        {
          LOG(ERROR) << "unloading error for client " << clientId;
        }
      }
    }

    bool unload(const std::string &rr, const std::string &id)
    {
      LOG(INFO) << "UNLOADING";
      map[rr]++;
      return true;
    }

  private:
    std::map<std::string, int> &map;
  };

  {
    UnloadTestRr rr("rr", unloadCounter);

    RrCatalog catalog;
    catalog.storeClientCallRecord(IDUtils::generateServerName("targetRr", "targetServer"), "id1");
    catalog.storeClientCallRecord(IDUtils::generateServerName("targetRr", "targetServer"), "id2");
    catalog.storeClientCallRecord(IDUtils::generateServerName("targetRr", "targetServer"), "id3");
    catalog.storeClientCallRecord(IDUtils::generateServerName("targetRr2", "targetServer2"), "id4");

    // used for destructor testing
    catalog.storeClientCallRecord(IDUtils::generateServerName("targetRr3", "targetServer3"), "id5");

    rr.updateCatalog(catalog);

    EXPECT_EQ(rr.clientCount(), 0);

    std::string targetRr = "targetRr";
    std::string targetServer = "targetServer";
    std::string targetQuery = "targetQuery";
    rr.createClient<RrClientBase, std::string, void *const>(targetRr, targetServer, targetQuery);

    EXPECT_EQ(rr.clientCount(), 1);

    targetRr = "targetRr2";
    targetServer = "targetServer2";
    targetQuery = "targetQuery2";
    rr.createClient<RrClientBase, std::string, void *const>(targetRr, targetServer, targetQuery);

    EXPECT_EQ(rr.clientCount(), 2);

    std::string unloadTarget = IDUtils::generateServerName("targetRr", "targetServer");
    rr.unloadClient(unloadTarget);

    EXPECT_EQ(rr.clientCount(), 1);
    EXPECT_EQ(unloadCounter["targetRr"], 3);

    unloadTarget = IDUtils::generateServerName("targetRr2", "targetServer2");
    rr.unloadClient(unloadTarget);

    EXPECT_EQ(rr.clientCount(), 0);
    EXPECT_EQ(unloadCounter["targetRr2"], 1);

    LOG(INFO) << "checking invalid client scenario";

    EXPECT_THROW({
      try
      {
        unloadTarget = IDUtils::generateServerName("targetRr2", "targetServer3");
        rr.unloadClient(unloadTarget);
      }
      catch (const ElementNotFoundException &e)
      {
        EXPECT_STREQ("unloadClient targetRr2/targetServer3 not found", e.what());
        throw;
      }
    },
                 ElementNotFoundException);

    EXPECT_EQ(rr.clientCount(), 0);

    targetRr = "targetRr3";
    targetServer = "targetServer3";
    targetQuery = "targetQuery3";
    rr.createClient<RrClientBase, std::string, void *const>(targetRr, targetServer, targetQuery);

    EXPECT_EQ(rr.clientCount(), 1);
  }

  EXPECT_EQ(unloadCounter["targetRr3"], 1);
}

int s1loadCount, s1unloadCount, s2loadCount, s2unloadCount = 0;

void S1LoadCb(RrQueryTemplate<Resource1> &query)
{
  s1loadCount++;

  LOG(INFO)
      << "S1LoadCb called";

  LOG(INFO) << "executing normal call...";
  RrQueryRequestTemplate<Resource2> req(Resource2(1, 0));
  RrQueryResponseTemplate<Resource2> resp(Resource2(0, 0));
  RrQueryTemplate<Resource2> newQuery(req, resp);

  rr_m1.call<RrTemplateServer<Resource2>, RrQueryTemplate<Resource2>>(rr_m2, "R2_S_e", newQuery);

  LOG(INFO) << "executing error call...";
  RrQueryRequestTemplate<Resource2> req2(Resource2(2, 0));
  RrQueryResponseTemplate<Resource2> resp2(Resource2(0, 0));
  RrQueryTemplate<Resource2> newQuery2(req2, resp2);

  rr_m1.call<RrTemplateServer<Resource2>, RrQueryTemplate<Resource2>>(rr_m2, "R2_S_e", newQuery2);
};

void S1UnloadCb(RrQueryTemplate<Resource1> &query)
{
  s1unloadCount++;
  LOG(INFO) << "S1UnloadCb called";
};

void S2LoadCb(RrQueryTemplate<Resource2> &query)
{
  s2loadCount++;
  LOG(INFO) << "RtM2LoadCB called";
  if (query.request().getRequest().i_ == 2)
    throw resource_registrar::TemotoErrorStack("S2 encountered an error", "R2_S_e");
  else
    LOG(INFO) << "NO error to throw yet";
};

void S2UnloadCb(RrQueryTemplate<Resource2> &query)
{
  s2unloadCount++;
  LOG(INFO) << "RtM2UnloadCB called";
};

TEST_F(RrBaseTest, CallbackErrorTest)
{
  LOG(INFO) << "Registering servers that throw errors in callback";
  rr_m1.registerServer(std::make_unique<RrTemplateServer<Resource1>>("R1_S_e", &S1LoadCb, &S1UnloadCb));
  rr_m2.registerServer(std::make_unique<RrTemplateServer<Resource2>>("R2_S_e", &S2LoadCb, &S2UnloadCb));

  EXPECT_EQ(s1loadCount, 0);
  EXPECT_EQ(s1unloadCount, 0);
  EXPECT_EQ(s2loadCount, 0);
  EXPECT_EQ(s2unloadCount, 0);

  RrQueryTemplate<Resource1> query(Resource1("throw error please"), Resource1(""));

  LOG(INFO) << "calling expecting an exception";
  try
  {
    rr_m0.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_m1, "R1_S_e", query);
  }
  catch (const resource_registrar::TemotoErrorStack &e)
  {

    EXPECT_EQ(s1loadCount, 1);
    EXPECT_EQ(s1unloadCount, 0);
    EXPECT_EQ(s2loadCount, 2);
    EXPECT_EQ(s2unloadCount, 1);

    LOG(INFO) << "Checking exception stack length...";
    EXPECT_EQ(e.getErrorStack().size(), 3);

    LOG(INFO) << "Checking exception stack contents";
    for (const resource_registrar::TemotoError &stackElement : e.getErrorStack())
    {
      if (stackElement.getOrigin() == "R2_S_e")
      {
        EXPECT_EQ(stackElement.getMessage(), "S2 encountered an error");
      }
      else
      {
        EXPECT_EQ(stackElement.getMessage(), "forwarding");
      }
    }
  }
  catch (...)
  {
    FAIL() << "Unexpected error happened while handling erronous callback";
  }
  LOG(INFO) << "Exception system appears to work";
}

TEST_F(RrBaseTest, DataFetchTest)
{

  LOG(INFO) << "constructing test RRs";
  RrBase rr_cli("rr_client");
  RrBase rr_agnt("rr_agent");
  RrBase rr_srv("rr_server");
  std::unordered_map<std::string, RrBase *> rr_ref;
  rr_ref["rr_client"] = &rr_cli;
  rr_ref["rr_server"] = &rr_srv;
  rr_ref["rr_agent"] = &rr_agnt;
  rr_cli.setRrReferences(rr_ref);
  rr_srv.setRrReferences(rr_ref);
  rr_agnt.setRrReferences(rr_ref);

  std::vector<std::string> childQueryIds;

  auto loadCb = [&](RrQueryTemplate<Resource1> &query) {
    LOG(INFO) << "S1LoadCb called";

    RrQueryTemplate<Resource2> childQuery(Resource2(1, 2), Resource2());
    rr_agnt.call<RrTemplateServer<Resource2>, RrQueryTemplate<Resource2>>(rr_srv, "srv2", childQuery);
    childQueryIds.push_back(childQuery.id());

    RrQueryTemplate<Resource2> childQuery2(Resource2(2, 2), Resource2());
    rr_agnt.call<RrTemplateServer<Resource2>, RrQueryTemplate<Resource2>>(rr_srv, "srv2", childQuery2);
    childQueryIds.push_back(childQuery2.id());
  };

  auto unloadCb = [&](RrQueryTemplate<Resource1> &query) {
    LOG(INFO) << "S1UnLoadCb called";
  };

  auto loadCb2 = [&](RrQueryTemplate<Resource2> &query) {
    LOG(INFO) << "S2LoadCb called";
  };

  auto unloadCb2 = [&](RrQueryTemplate<Resource2> &query) {
    LOG(INFO) << "S2UnLoadCb called";
  };

  LOG(INFO) << "registering server named 'srv'";
  rr_agnt.registerServer(std::make_unique<RrTemplateServer<Resource1>>("srv", loadCb, unloadCb, 1));

  rr_srv.registerServer(std::make_unique<RrTemplateServer<Resource2>>("srv2", loadCb2, unloadCb2, 1));

  LOG(INFO) << "executing call";
  RrQueryTemplate<Resource1> query(Resource1("someQueryContent"), Resource1(""));
  rr_cli.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_agnt, "srv", query);

  //RrQueryTemplate<Resource1> query2(Resource1("someQueryContent2"), Resource1(""));
  //rr_cli.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_agnt, "srv", query2);

  LOG(INFO) << "fetching queries";

  rr_cli.printCatalog();
  LOG(INFO) << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
  rr_agnt.printCatalog();
  LOG(INFO) << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
  rr_srv.printCatalog();

  std::map<std::string, std::pair<std::string, std::string>> resultMap = rr_agnt.getChildQueries(query.id(), "srv2");
  EXPECT_EQ(resultMap.size(), childQueryIds.size());

  int i = 1;
  for (const auto &el : childQueryIds)
  {
    EXPECT_EQ(resultMap.count(el), 1);


    Resource2 query = Serializer::deserialize<Resource2>(resultMap[el].first);
    EXPECT_EQ(query.i_, i);

    i++;
  }
}

TEST_F(RrBaseTest, RrServerStatusCallbackTest)
{

  LOG(INFO) << "constructing test RRs";
  RrBase rr_cli("rr_client");
  RrBase rr_srv("rr_server");
  std::unordered_map<std::string, RrBase *> rr_ref;
  rr_ref["rr_client"] = &rr_cli;
  rr_ref["rr_server"] = &rr_srv;
  rr_cli.setRrReferences(rr_ref);
  rr_srv.setRrReferences(rr_ref);

  int statusCbCnt = 0;

  auto loadCb = [&](RrQueryTemplate<Resource1> &query) {
    LOG(INFO) << "S1LoadCb called";
  };

  auto unloadCb = [&](RrQueryTemplate<Resource1> &query) {
    LOG(INFO) << "S1UnLoadCb called";
  };

  auto statusCb = [&](Resource1 res, const Status &status) {
    LOG(INFO) << "Status CB called";

    EXPECT_EQ(res.rawMessage(), "someQueryContent");
    EXPECT_EQ(status.state_, Status::State::FATAL);
    EXPECT_EQ(status.message_, "message");
    statusCbCnt++;
  };

  LOG(INFO) << "registering server named 'srv'";
  rr_srv.registerServer(std::make_unique<RrTemplateServer<Resource1>>("srv", loadCb, unloadCb, statusCb));

  LOG(INFO) << "executing call";
  RrQueryTemplate<Resource1> query(Resource1("someQueryContent"), Resource1(""));
  rr_cli.call<RrTemplateServer<Resource1>, RrQueryTemplate<Resource1>>(rr_srv, "srv", query);

  std::string id = query.id();
  LOG(INFO) << "\n\n\n";
  LOG(INFO) << "call returned id: " << id;

  rr_srv.sendStatus(id, {Status::State::FATAL, id, "message"});

  EXPECT_EQ(statusCbCnt, 1);
}