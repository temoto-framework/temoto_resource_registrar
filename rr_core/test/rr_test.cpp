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

  std::string requestId_;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /* version */)
  {
    ar &requestId_ &typed_request_ &typed_response_;
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
      MessageType q = Serializer::deserialize<MessageType>(status.serialisedRequest_);
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