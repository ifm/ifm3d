// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_TIME_H
#define IFM3D_COMMON_LOGGING_TIME_H

#include <chrono>

namespace ifm3d
{
  using logging_clock = std::chrono::system_clock;
  using logging_timepoint = std::chrono::time_point<std::chrono::system_clock>;
}

#endif // IFM3D_COMMON_LOGGING_TIME_H
