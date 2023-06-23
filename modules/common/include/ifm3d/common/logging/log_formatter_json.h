// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_FORMATTER_JSON_H
#define IFM3D_COMMON_LOGGING_LOG_FORMATTER_JSON_H

#include <cstring>

#include <fmt/format.h>

#include <ifm3d/common/json.hpp>
#include <ifm3d/common/logging/log_entry.h>

namespace ifm3d
{
  class LogFormatterJson
  {
  public:
    static std::string
    format(const LogEntry& entry)
    {

      return json::object(
               {
                 {"time",
                  fmt::format("{:%Y-%m-%dT%H:%M:%S%z}", entry.GetTime())},
                 {"msg", entry.GetMessage()},
                 {"file",
                  fmt::format("{}:{}", entry.GetFile(), entry.GetLine())},
                 {"level", LogLevelToString(entry.GetLogLevel())},
               })
        .dump();

      return {};
    }
  };
}
#endif // IFM3D_COMMON_LOGGING_LOG_FORMATTER_JSON_H
