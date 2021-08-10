/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SWUPDATER_SWUPDATE_APP_H
#define IFM3D_TOOLS_SWUPDATER_SWUPDATE_APP_H

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `swupdate` subcommand to the
   * `ifm3d` command-line utility.
   */
  class SWUpdateApp : public ifm3d::CmdLineApp
  {
  public:
    SWUpdateApp(int argc,
                const char** argv,
                const std::string& name = "swupdate");
    int Run();
  };

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_SWUPDATER_SWUPDATE_APP_H
