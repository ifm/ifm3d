// -*- c++ -*-
/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __IFM3D_TOOLS_JITTER_APP_H__
#define __IFM3D_TOOLS_JITTER_APP_H__

#include <string>
#include <ifm3d/tools/fg/fg_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `jitter` subcommand to the `ifm3d`
   * command-line utility.
   */
  class JitterApp : public ifm3d::FgApp
  {
  public:
    JitterApp(int argc, const char** argv, const std::string& name = "jitter");
    int Run();
  };

} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_JITTER_APP_H__
