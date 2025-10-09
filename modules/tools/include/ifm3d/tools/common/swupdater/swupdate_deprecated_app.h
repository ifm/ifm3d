/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SWUPDATER_SWUPDATE_DEPRECATED_APP_H
#define IFM3D_TOOLS_SWUPDATER_SWUPDATE_DEPRECATED_APP_H
#pragma once

#include <ifm3d/common/features.h>
#if defined(BUILD_MODULE_SWUPDATER)

#  include <ifm3d/tools/command.hpp>
#  include <ifm3d/tools/main_command.hpp>
#  include <string>

namespace ifm3d
{
  /**
   * Concrete implementation of the deprecated `swupdate` subcommand to the
   * `ifm3d` command-line utility.
   */
  class SWUpdateDeprecatedApp : public Command
  {
  public:
    SWUpdateDeprecatedApp() = default;
    SWUpdateDeprecatedApp(const SWUpdateDeprecatedApp&) = default;
    SWUpdateDeprecatedApp(SWUpdateDeprecatedApp&&) = delete;
    SWUpdateDeprecatedApp& operator=(const SWUpdateDeprecatedApp&) = default;
    SWUpdateDeprecatedApp& operator=(SWUpdateDeprecatedApp&&) = delete;
    ~SWUpdateDeprecatedApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

    std::string swu_file{"-"};
    bool check{false};
    bool reboot{false};
    bool quiet{false};
    unsigned short sw_port{8080};
    unsigned int timeout{600};

  }; // end: class SWUpdateDeprecatedApp

} // end: namespace ifm3d

#endif
#endif // IFM3D_TOOLS_SWUPDATER_SWUPDATE_DEPRECATED_APP_H
