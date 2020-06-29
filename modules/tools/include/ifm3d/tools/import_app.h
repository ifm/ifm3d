// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_TOOLS_IMPORT_APP_H__
#define __IFM3D_TOOLS_IMPORT_APP_H__

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `import` subcommand to the `ifm3d`
   * command-line utility.
   *
   * `import` provides compatibility with Vision Assistant - it can consume
   * application or configurations exported by Vision Assistant.
   */
  class ImportApp : public ifm3d::CmdLineApp
  {
  public:
    ImportApp(int argc, const char** argv, const std::string& name = "import");
    int Run();
  }; // end: class ImportApp

} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_IMPORT_APP_H__
