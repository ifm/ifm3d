// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_WRITER_COMPOSITE_H
#define IFM3D_COMMON_LOGGING_LOG_WRITER_COMPOSITE_H

#include <ifm3d/common/logging/log_writer.h>

namespace ifm3d
{
  class LogWriterComposite : public LogWriter
  {
  public:
    LogWriterComposite(std::vector<std::shared_ptr<LogWriter>> writers)
      : writers_(writers)
    {}

    void
    Write(const LogEntry& entry) override
    {
      for (const auto& it : this->writers_)
        {
          it->Write(entry);
        }
    }

  protected:
    std::vector<std::shared_ptr<LogWriter>> writers_;
  };
}
#endif // IFM3D_COMMON_LOGGING_LOG_WRITER_COMPOSITE_H
