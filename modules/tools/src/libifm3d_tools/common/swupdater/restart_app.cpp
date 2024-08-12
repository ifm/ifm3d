/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/common/swupdater/restart_app.h>
#include <exception>
#include <iostream>
#include <istream>
#include <fstream>
#include <memory>
#include <vector>
#include <chrono>
#include <filesystem>
#include <ifm3d/device.h>
#include <ifm3d/swupdater.h>

#ifdef _WIN32
#  include <io.h>
#  include <fcntl.h>
#endif

ifm3d::RestartApp::~RestartApp() {}

void
ifm3d::RestartApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice(false);
  ifm3d::SWUpdater::Ptr swupdater = std::make_shared<ifm3d::SWUpdater>(device);

  if (swupdater->WaitForRecovery(-1))
    {
      swupdater->RebootToRecovery();
    }
  else
    {
      device->Reboot();
    }
}

CLI::App*
ifm3d::RestartApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command = parent->add_subcommand("restart", "Restart the device.")
                        ->require_subcommand(0, 0);

// Hide command if SWUpdate module is not build
#if !defined(BUILD_MODULE_SWUPDATER)
  swupdate_cmd->group("");
#endif

  return command;
}

bool
ifm3d::RestartApp::CheckCompatibility()
{
  return true;
}