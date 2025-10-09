// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_LEVEL_H
#define IFM3D_COMMON_LOGGING_LOG_LEVEL_H

#include <cctype>
#include <cstdint>

namespace ifm3d
{
  enum class LogLevel : std::uint8_t
  {
    None = 0,
    Critical = 1,
    Error = 2,
    Warning = 3,
    Info = 4,
    Debug = 5,
    Verbose = 6
  };

  inline const char*
  log_level_to_string(LogLevel severity)
  {
    switch (severity)
      {
      case LogLevel::Critical:
        return "CRITICAL";
      case LogLevel::Error:
        return "ERROR";
      case LogLevel::Warning:
        return "WARN";
      case LogLevel::Info:
        return "INFO";
      case LogLevel::Debug:
        return "DEBUG";
      case LogLevel::Verbose:
        return "VERBOSE";
      default:
        return "NONE";
      }
  }

  inline LogLevel
  log_level_from_string(const char* str)
  {
    switch (std::toupper(str[0]))
      {
      case 'C':
        return LogLevel::Critical;
      case 'E':
        return LogLevel::Error;
      case 'W':
        return LogLevel::Warning;
      case 'I':
        return LogLevel::Info;
      case 'D':
        return LogLevel::Debug;
      case 'V':
        return LogLevel::Verbose;
      default:
        return LogLevel::None;
      }
  }
}
#endif // IFM3D_COMMON_LOGGING_LOG_LEVEL_H
