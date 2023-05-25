
#include <gtest/gtest.h>

int
main(int argc, char** argv)
{
  std::map<std::string, std::string> device_to_filter(
    {{"device_independent", "Logging.*"}});
  std::string gtest_filter = device_to_filter["device_independent"];
  ::testing::GTEST_FLAG(filter) = gtest_filter.c_str();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
