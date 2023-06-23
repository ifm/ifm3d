// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_WRITER_CONSOLE_COLORED_H
#define IFM3D_COMMON_LOGGING_LOG_WRITER_CONSOLE_COLORED_H

#include <ifm3d/common/logging/log_writer_console.h>

namespace ifm3d
{
  template <class Formatter,
            typename std::enable_if_t<std::is_same_v<
              decltype(Formatter::format(
                ifm3d::LogEntry("", ifm3d::LogLevel::Info, "", "", 1))),
              std::string>>* = nullptr>
  class LogWriterConsoleColored : public LogWriterConsole<Formatter>
  {
  public:
    LogWriterConsoleColored(Output out = Output::StdErr)
      : LogWriterConsole<Formatter>(out)
    {}

    void
    Write(const LogEntry& entry) override
    {
      if (this->is_a_tty_)
        {
          const auto str = Formatter::format(entry);
          const std::lock_guard<std::mutex> lock(this->mutex_);
          this->SetColor(entry.GetLogLevel());
          this->out_ << str;
          this->ResetColor();
          this->out_ << std::endl;
        }
      else
        {
          LogWriterConsole<Formatter>::Write(entry);
        }
    }

  protected:
    void
    SetColor(LogLevel log_level)
    {
      switch (log_level)
        {
        case LogLevel::Critical:
          this->out_ << "\x1B[97m\x1B[41m"; // white on red background
          break;

        case LogLevel::Error:
          this->out_ << "\x1B[91m"; // red
          break;

        case LogLevel::Warning:
          this->out_ << "\x1B[93m"; // yellow
          break;

        case LogLevel::Debug:
        case LogLevel::Verbose:
          this->out_ << "\x1B[96m"; // cyan
          break;
        default:
          break;
        }
    }

    void
    ResetColor()
    {
      this->out_ << "\x1B[0m\x1B[0K";
    }
  };
}
#endif // IFM3D_COMMON_LOGGING_LOG_WRITER_CONSOLE_COLORED_H
