// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_FORMATTER_TEXT_H
#define IFM3D_COMMON_LOGGING_LOG_FORMATTER_TEXT_H

#include <cstring>
#include <ifm3d/common/logging/log_entry.h>
#include <ifm3d/common/common_export.h>
namespace ifm3d
{
  class IFM3D_COMMON_EXPORT LogFormatterText
  {
  public:
    static std::string format(const LogEntry& entry);
  };
}
#endif // IFM3D_COMMON_LOGGING_LOG_FORMATTER_TEXT_H
