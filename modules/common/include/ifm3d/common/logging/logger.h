// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOGGER_H
#define IFM3D_COMMON_LOGGING_LOGGER_H

#include <ifm3d/common/logging/log_level.h>
#include <ifm3d/common/logging/log_writer.h>
#include <ifm3d/common/logging/log_writer_console.h>
#include <ifm3d/common/logging/log_writer_console_colored.h>
#include <ifm3d/common/logging/log_formatter_text.h>
#include <ifm3d/common/common_export.h>

#include <vector>
#include <memory>

namespace ifm3d
{
  class IFM3D_COMMON_EXPORT Logger
  {
  public:
    Logger(LogLevel log_level = LogLevel::Warning)
      : log_level_(log_level),
        writer_(std::make_shared<LogWriterConsoleColored<LogFormatterText>>(
          Output::StdErr))
    {}

    Logger&
    SetWriter(std::shared_ptr<LogWriter> writer)
    {
      this->writer_ = writer;
      return *this;
    }

    LogLevel
    GetLogLevel() const
    {
      return this->log_level_;
    }

    void
    SetLogLevel(LogLevel log_level)
    {
      this->log_level_ = log_level;
    }

    inline bool
    ShouldLog(LogLevel log_level) const
    {
      return log_level <= this->log_level_;
    }

    inline void
    Write(const LogEntry& entry)
    {
      if (this->writer_)
        {
          this->writer_->Write(entry);
        }
    }

    static Logger& Get();

  private:
    LogLevel log_level_;
    std::shared_ptr<LogWriter> writer_;
  };

}

#endif // IFM3D_COMMON_LOGGING_LOGGER_H