
#include <gtest/gtest.h>
#include <map>
#include <string>

int
main(int argc, char** argv)
{
  std::map<std::string, std::string> device_to_filter(
    {{"device_independent", "Logging.*"}});
  std::string const gtest_filter = device_to_filter["device_independent"];
  ::testing::GTEST_FLAG(filter) = gtest_filter;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
