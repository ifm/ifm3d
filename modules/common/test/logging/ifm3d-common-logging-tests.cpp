#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <algorithm>
#include "gtest/gtest.h"
#include <ifm3d/common/logging/logger.h>

TEST(Logging, instance)
{
  EXPECT_NO_FATAL_FAILURE(auto& logger = ifm3d::Logger::Get());
}

TEST(Logging, get_default_value)
{
  auto& logger = ifm3d::Logger::Get();
  EXPECT_TRUE(logger.GetLogLevel() == ifm3d::LogLevel::Warning);
}

TEST(Logging, set_log_value)
{
  auto& logger = ifm3d::Logger::Get();
  EXPECT_NO_FATAL_FAILURE(logger.SetLogLevel(ifm3d::LogLevel::Info));
}

TEST(Logging, get_log_value)
{
  auto& logger = ifm3d::Logger::Get();
  EXPECT_TRUE(logger.GetLogLevel() == ifm3d::LogLevel::Info);
}

TEST(Logging, shouldlog)
{
  auto& logger = ifm3d::Logger::Get();

  EXPECT_NO_FATAL_FAILURE(logger.SetLogLevel(ifm3d::LogLevel::Info));
  EXPECT_TRUE(logger.ShouldLog(ifm3d::LogLevel::Info));
  EXPECT_TRUE(logger.ShouldLog(ifm3d::LogLevel::Warning));
  EXPECT_FALSE(logger.ShouldLog(ifm3d::LogLevel::Verbose));
}
