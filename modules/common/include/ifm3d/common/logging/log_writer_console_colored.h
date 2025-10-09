// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_WRITER_CONSOLE_COLORED_H
#define IFM3D_COMMON_LOGGING_LOG_WRITER_CONSOLE_COLORED_H

#include <ifm3d/common/logging/log_level.h>
#include <ifm3d/common/logging/log_writer_console.h>
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  define NOMINMAX
#  include <windows.h>
#  undef GetMessage
#endif

namespace ifm3d
{
  template <class FORMATTER,
            typename std::enable_if_t<std::is_same_v<
              decltype(FORMATTER::Format(
                ifm3d::LogEntry("", ifm3d::LogLevel::Info, "", "", 1))),
              std::string>>* = nullptr>
  class LogWriterConsoleColored : public LogWriterConsole<FORMATTER>
  {
  public:
    LogWriterConsoleColored(Output out = Output::StdErr)
      : LogWriterConsole<FORMATTER>(out)
    {
#ifdef _WIN32
      // NOLINTNEXTLINE(*-prefer-member-initializer)
      _colored_output_available =
        this->_is_a_tty && EnableVirtualTerminalProcessing();
#else
      // NOLINTNEXTLINE(*-prefer-member-initializer)
      _colored_output_available = this->_is_a_tty;
#endif
    }

    void
    Write(const LogEntry& entry) override
    {
      if (this->_colored_output_available)
        {
          const auto str = FORMATTER::Format(entry);
          const std::lock_guard<std::mutex> lock(this->_mutex);
          this->set_color(entry.GetLogLevel());
          this->_out << str;
          this->reset_color();
          this->_out << std::endl;
        }
      else
        {
          LogWriterConsole<FORMATTER>::Write(entry);
        }
    }

  protected:
    bool _colored_output_available = false;
#ifdef _WIN32
    bool
    EnableVirtualTerminalProcessing()
    {
      HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
      if (hOut == INVALID_HANDLE_VALUE)
        {
          return false;
        }

      DWORD dwMode = 0;
      if (!GetConsoleMode(hOut, &dwMode))
        {
          return false;
        }

      dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
      if (!SetConsoleMode(hOut, dwMode))
        {
          return false;
        }
      return true;
    }
#endif

    void
    set_color(LogLevel log_level)
    {
      switch (log_level)
        {
        case LogLevel::Critical:
          this->_out << "\x1B[97m\x1B[41m"; // white on red background
          break;

        case LogLevel::Error:
          this->_out << "\x1B[91m"; // red
          break;

        case LogLevel::Warning:
          this->_out << "\x1B[93m"; // yellow
          break;

        case LogLevel::Debug:
        case LogLevel::Verbose:
          this->_out << "\x1B[96m"; // cyan
          break;
        default:
          break;
        }
    }

    void
    reset_color()
    {
      this->_out << "\x1B[0m\x1B[0K";
    }
  };
}
#endif // IFM3D_COMMON_LOGGING_LOG_WRITER_CONSOLE_COLORED_H
