#include <fmt/format.h>
#include <fmt/chrono.h>
#include <ifm3d/common/logging/logger.h>

ifm3d::Logger&
ifm3d::Logger::Get()
{
  static Logger instance;
  return instance;
}
std::string
ifm3d::LogFormatterText::format(const LogEntry& entry)
{
  return fmt::format("{} {} [{}:{}] {}",
                     entry.GetTime(),
                     LogLevelToString(entry.GetLogLevel()),
                     entry.GetFile(),
                     entry.GetLine(),
                     entry.GetMessage());
}