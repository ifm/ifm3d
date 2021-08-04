// -*- c++ -*-
/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_PASSWD_APP_H
#define IFM3D_TOOLS_PASSWD_APP_H

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `reboot` subcommand to the `ifm3d`
   * command-line utility.
   */
  class PasswdApp : public ifm3d::CmdLineApp
  {
  public:
    PasswdApp(int argc,
              const char** argv,
              const std::string& name = "password");
    int Run() override;
    bool CheckCompatibility() override;
  }; // end: class Passwd
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_PASSWD_APP_H
