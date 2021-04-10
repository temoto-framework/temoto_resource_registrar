#include "glog/logging.h"
#include "gtest/gtest.h"

#include <thread>

int main(int argc, char **argv)
{

  LOG(INFO) << std::this_thread::get_id();

  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  LOG(INFO) << "Initializing GoogleTest";
  ::testing::InitGoogleTest(&argc, argv);
  LOG(INFO) << "Running GoogleTest";
  int ret = RUN_ALL_TESTS();
  return ret;
}