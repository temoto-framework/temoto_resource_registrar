#include "glog/logging.h"
#include "gtest/gtest.h"

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  LOG(INFO) << "Initializing GoogleTest";
  ::testing::InitGoogleTest(&argc, argv);
  LOG(INFO) << "Running GoogleTest";
  int ret = RUN_ALL_TESTS();
  return ret;
}