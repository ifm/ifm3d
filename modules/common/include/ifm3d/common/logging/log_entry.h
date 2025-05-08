// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_ENTRY_H
#define IFM3D_COMMON_LOGGING_LOG_ENTRY_H

#include <cstdarg>
#include <string>
#include <utility>

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
      : _time(logging_clock::now()),
        _log_level(log_level),
        _message(std::move(message)),
        _line(line),
        _func(function),
        _file(file)
    {}

    [[nodiscard]] const logging_timepoint&
    GetTime() const
    {
      return _time;
    }

    [[nodiscard]] LogLevel
    GetLogLevel() const
    {
      return _log_level;
    }

    [[nodiscard]] size_t
    GetLine() const
    {
      return _line;
    }

    [[nodiscard]] const char*
    GetMessage() const
    {
      return _message.c_str();
    }

    [[nodiscard]] const char*
    GetFunc() const
    {
      return _func;
    }

    [[nodiscard]] const char*
    GetFile() const
    {
      return _file;
    }

  private:
    logging_timepoint _time;
    LogLevel _log_level;
    std::string _message;
    size_t _line;
    const char* _func;
    const char* _file;
  };
}

#endif // IFM3D_COMMON_LOGGING_LOG_ENTRY_H
