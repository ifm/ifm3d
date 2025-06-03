// -*- c++ -*-
/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_UTIL_H
#define IFM3D_COMMON_UTIL_H

#include <iostream>
#include <string>

#if defined(_MSC_VER)
#  include <io.h>
#  include <windows.h>
#  define IFM3D_IS_A_TTY(stream) (!!_isatty(_fileno(stream)))
#else
#  include <unistd.h>
#  include <termios.h>
#  define IFM3D_IS_A_TTY(stream) (!!isatty(fileno(stream)))
#endif

namespace ifm3d
{
  void stdin_echo(bool enable = true);
  std::string read_password();
}

#endif
