// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_WRITER_H
#define IFM3D_COMMON_LOGGING_LOG_WRITER_H

#include <ifm3d/common/logging/log_entry.h>

namespace ifm3d
{
  class LogWriter
  {
  public:
    virtual ~LogWriter() {}
    virtual void Write(const LogEntry& entry) = 0;
  };
}
#endif // IFM3D_COMMON_LOGGING_LOG_WRITER_H