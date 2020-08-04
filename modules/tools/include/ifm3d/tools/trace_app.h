// -*- c++ -*-
/*
 * Copyright (C) 2018 ifm syntron gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_TOOLS_TRACE_APP_H__
#define __IFM3D_TOOLS_TRACE_APP_H__

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `trace` subcommand to the `ifm3d`
   * command-line utility.
   */
  class TraceApp : public ifm3d::CmdLineApp
  {
  public:
    TraceApp(int argc, const char** argv, const std::string& name = "trace");
    int Run();
  }; // end: class TraceApp
} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_TRACE_APP_H__
