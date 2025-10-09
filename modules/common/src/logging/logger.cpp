#include <ctime>
#include <fmt/chrono.h> // NOLINT(*)
#include <fmt/core.h>   // NOLINT(misc-header-include-cycle)
#include <ifm3d/common/logging/log_entry.h>
#include <ifm3d/common/logging/log_formatter_text.h>
#include <ifm3d/common/logging/log_level.h>
#include <ifm3d/common/logging/logger.h>
#include <string>

ifm3d::Logger&
ifm3d::Logger::Get()
{
  static Logger INSTANCE;
  return INSTANCE;
}
std::string
ifm3d::LogFormatterText::Format(const LogEntry& entry)
{
  return fmt::format("{} {} [{}:{}] {}",
                     entry.GetTime(),
                     log_level_to_string(entry.GetLogLevel()),
                     entry.GetFile(),
                     entry.GetLine(),
                     entry.GetMessage());
}