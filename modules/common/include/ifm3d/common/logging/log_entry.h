// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_ENTRY_H
#define IFM3D_COMMON_LOGGING_LOG_ENTRY_H

#include <cstdarg>
#include <thread>
#include <string>

#include <ifm3d/common/logging/log_level.h>
#include <ifm3d/common/logging/time.h>

namespace ifm3d
{
  class LogEntry
  {
  public:
    LogEntry(std::string message,
             LogLevel log_level,
             const char* file,
             const char* function,
             size_t line)
      : message_(message),
        log_level_(log_level),
        file_(file),
        func_(function),
        line_(line),
        time_(logging_clock::now())
    {}

    const logging_timepoint&
    GetTime() const
    {
      return time_;
    }

    LogLevel
    GetLogLevel() const
    {
      return log_level_;
    }

    size_t
    GetLine() const
    {
      return line_;
    }

    const char*
    GetMessage() const
    {
      return message_.c_str();
    }

    const char*
    GetFunc() const
    {
      return func_;
    }

    const char*
    GetFile() const
    {
      return file_;
    }

  private:
    logging_timepoint time_;
    const LogLevel log_level_;
    std::string message_;
    const size_t line_;
    const char* const func_;
    const char* const file_;
  };
}

#endif // IFM3D_COMMON_LOGGING_LOG_ENTRY_H
