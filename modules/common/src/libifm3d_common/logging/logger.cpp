#include "ifm3d/common/logging/log_formatter_text.h"
#include "ifm3d/common/logging/log_entry.h"
#include "fmt/core.h"
#include "ifm3d/common/logging/log_level.h"
#include <ifm3d/common/logging/logger.h>
#include <string>
#include <ctime>
#include <fmt/chrono.h> // NOLINT(*)

ifm3d::Logger&
ifm3d::Logger::Get()
{
  static Logger INSTANCE;
  return INSTANCE;
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