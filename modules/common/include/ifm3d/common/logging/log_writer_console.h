// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_WRITER_CONSOLE_H
#define IFM3D_COMMON_LOGGING_LOG_WRITER_CONSOLE_H

#include <mutex>
#include <cstring>
#include <cstdio>
#include <iostream>

#if defined(_MSC_VER)
#  include <io.h>
#  define IS_A_TTY(stream) (!!_isatty(_fileno(stream)))
#else
#  include <unistd.h>
#  define IS_A_TTY(stream) (!!isatty(fileno(stream)))
#endif

#include <ifm3d/common/logging/log_writer.h>

namespace ifm3d
{
  enum class Output
  {
    StdOut,
    StdErr,
  };

  template <class Formatter,
            typename std::enable_if_t<std::is_same_v<
              decltype(Formatter::format(
                ifm3d::LogEntry("", ifm3d::LogLevel::Info, "", "", 1))),
              std::string>>* = nullptr>
  class LogWriterConsole : public LogWriter
  {
  public:
    LogWriterConsole(Output out = Output::StdErr)
      : out_(out == Output::StdOut ? std::cout : std::cerr),
        is_a_tty_(IS_A_TTY(out == Output::StdOut ? stdout : stderr))
    {}

    void
    Write(const LogEntry& entry) override
    {
      const auto str = Formatter::format(entry);
      const std::lock_guard<std::mutex> lock(this->mutex_);
      this->out_ << str << std::endl;
    }

  protected:
    std::mutex mutex_;
    std::ostream& out_;
    bool is_a_tty_;
  };
}
#endif // IFM3D_COMMON_LOGGING_LOG_WRITER_CONSOLE_H
