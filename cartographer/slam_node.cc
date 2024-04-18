#include <glog/logging.h>
#include <gtest/gtest.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  //   testing::InitGoogleTest(&argc, argv);
  return 0;
  //   return RUN_ALL_TESTS();
}