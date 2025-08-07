// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOGGER_H
#define IFM3D_COMMON_LOGGING_LOGGER_H

#include <ifm3d/common/logging/log_formatter_text.h>
#include <ifm3d/common/logging/log_level.h>
#include <ifm3d/common/logging/log_writer.h>
#include <ifm3d/common/logging/log_writer_console.h>
#include <ifm3d/common/logging/log_writer_console_colored.h>
#include <ifm3d/common/module_common.h>

#include <memory>

namespace ifm3d
{
  class IFM3D_EXPORT Logger
  {
  public:
    Logger(LogLevel log_level = LogLevel::Warning)
      : _log_level(log_level),
        _writer(std::make_shared<LogWriterConsoleColored<LogFormatterText>>(
          Output::StdErr))
    {}

    Logger&
    SetWriter(std::shared_ptr<LogWriter> writer)
    {
      this->_writer = writer;
      return *this;
    }

    [[nodiscard]] LogLevel
    GetLogLevel() const
    {
      return this->_log_level;
    }

    void
    SetLogLevel(LogLevel log_level)
    {
      this->_log_level = log_level;
    }

    [[nodiscard]] bool
    ShouldLog(LogLevel log_level) const
    {
      return log_level <= this->_log_level;
    }

    void
    Write(const LogEntry& entry)
    {
      if (this->_writer)
        {
          this->_writer->Write(entry);
        }
    }

    static Logger& Get();

  private:
    LogLevel _log_level;
    std::shared_ptr<LogWriter> _writer;
  };

}

#endif // IFM3D_COMMON_LOGGING_LOGGER_H