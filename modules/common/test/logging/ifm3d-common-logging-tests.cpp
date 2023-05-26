#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <algorithm>
#include "gtest/gtest.h"
#include <ifm3d/common/logging/logger.h>
#include <ifm3d/common/logging/log.h>

template <typename Formatter>
class Stringwriter : public ifm3d::LogWriter
{
public:
  void
  Write(const ifm3d::LogEntry& entry) override
  {
    log_line_print = Formatter::format(entry);
  }

  std::string log_line_print;
};

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

TEST(Logging, setlogger)
{
  auto& logger = ifm3d::Logger::Get();

  auto log_writer = std::make_shared<Stringwriter<ifm3d::LogFormatterText>>();
  EXPECT_NO_FATAL_FAILURE(logger.SetWriter(log_writer));
  EXPECT_NO_FATAL_FAILURE(logger.SetLogLevel(ifm3d::LogLevel::Info));

  LOG_VERBOSE("Test string")
  EXPECT_TRUE(log_writer->log_line_print.empty());

  LOG_INFO("Hello logger");

  EXPECT_FALSE(log_writer->log_line_print.empty());
  EXPECT_TRUE(log_writer->log_line_print.find("Hello logger") !=
              std::string::npos);
  EXPECT_TRUE(log_writer->log_line_print.find("INFO") != std::string::npos);
}
