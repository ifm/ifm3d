/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SWUPDATER_SWUPDATE_DEPRECATED_APP_H
#define IFM3D_TOOLS_SWUPDATER_SWUPDATE_DEPRECATED_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the deprecated `swupdate` subcommand to the
   * `ifm3d` command-line utility.
   */
  class SWUpdateDeprecatedApp : public Command
  {
  public:
    ~SWUpdateDeprecatedApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::string swu_file{"-"};
    bool check{false};
    bool reboot{false};
    bool quiet{false};
    unsigned short sw_port{8080};
    unsigned int timeout{600};

  }; // end: class SWUpdateDeprecatedApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_SWUPDATER_SWUPDATE_DEPRECATED_APP_H
