// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_WRITER_CONSOLE_H
#define IFM3D_COMMON_LOGGING_LOG_WRITER_CONSOLE_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ifm3d/common/logging/log_level.h>
#include <iostream>
#include <mutex>

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
  enum class Output : std::uint8_t
  {
    StdOut,
    StdErr,
  };

  template <class FORMATTER,
            typename std::enable_if_t<std::is_same_v<
              decltype(FORMATTER::Format(
                ifm3d::LogEntry("", ifm3d::LogLevel::Info, "", "", 1))),
              std::string>>* = nullptr>
  class LogWriterConsole : public LogWriter
  {
  public:
    LogWriterConsole(Output out = Output::StdErr)
      : _out(out == Output::StdOut ? std::cout : std::cerr),
        _is_a_tty(IS_A_TTY(out == Output::StdOut ? stdout : stderr))
    {}

    void
    Write(const LogEntry& entry) override
    {
      const auto str = FORMATTER::Format(entry);
      const std::lock_guard<std::mutex> lock(this->_mutex);
      this->_out << str << std::endl;
    }

  protected:
    std::mutex _mutex;
    std::ostream& _out; // NOLINT(*-avoid-const-or-ref-data-members)
    bool _is_a_tty;
  };
}
#endif // IFM3D_COMMON_LOGGING_LOG_WRITER_CONSOLE_H
