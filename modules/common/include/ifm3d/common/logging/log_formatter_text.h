// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_FORMATTER_TEXT_H
#define IFM3D_COMMON_LOGGING_LOG_FORMATTER_TEXT_H

#include <cstring>

#include <fmt/format.h>
#include <fmt/chrono.h>

#include <ifm3d/common/logging/log_entry.h>

namespace ifm3d
{
  class LogFormatterText
  {
  public:
    static std::string
    format(const LogEntry& entry)
    {

      return fmt::format("{} {} [{}:{}] {}",
                         entry.GetTime(),
                         LogLevelToString(entry.GetLogLevel()),
                         entry.GetFile(),
                         entry.GetLine(),
                         entry.GetMessage());
    }
  };
}
#endif // IFM3D_COMMON_LOGGING_LOG_FORMATTER_TEXT_H
