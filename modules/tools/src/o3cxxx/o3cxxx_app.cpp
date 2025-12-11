/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <ifm3d/common/features.h>
#include <ifm3d/device/device.h>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/common/discover_app.h>
#include <ifm3d/tools/common/fg/stat_app.h>
#include <ifm3d/tools/common/reboot_app.h>
#include <ifm3d/tools/common/reset_app.h>
#include <ifm3d/tools/common/swupdater/swupdate_app.h>
#include <ifm3d/tools/o3cxxx/o3cxxx_app.h>
#include <ifm3d/tools/ovp8xx/config_ovp8xx_app.h>
#include <ifm3d/tools/ovp8xx/diagnostic_app.h>
#include <ifm3d/tools/ovp8xx/get_service_report_app.h>
#include <ifm3d/tools/ovp8xx/sealedbox.h>

ifm3d::O3Cxxx::~O3Cxxx() = default;

CLI::App*
ifm3d::O3Cxxx::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("o3cxxx", "Interact with the O3C camera devices")
      ->require_subcommand(1);

  // Share the same subcommands as ovp8xx
  RegisterSubcommand<ifm3d::ConfigOvp8xxApp>(command);
  RegisterSubcommand<ifm3d::DiagnosticApp>(command);
  RegisterSubcommand<ifm3d::DiscoverApp>(command);
  RegisterSubcommand<ifm3d::ResetApp>(command)->SetDetails(
    "factoryReset",
    "Resets the device to its factory settings");
  RegisterSubcommand<ifm3d::GetServiceReportApp>(command);
#if defined(BUILD_MODULE_FRAMEGRABBER)
  RegisterSubcommand<ifm3d::StatApp>(command);
#endif
  RegisterSubcommand<ifm3d::RebootApp>(command);
#if defined(BUILD_MODULE_SWUPDATER)
  RegisterSubcommand<ifm3d::SWUpdateApp>(command,
                                         ifm3d::Device::SWUVersion::SWU_V2);
#endif
#if defined(BUILD_MODULE_CRYPTO)
  RegisterSubcommand<tools::ovp8xx::sealedbox::SealedBox>(command);
#endif
  return command;
}

void
ifm3d::O3Cxxx::Execute(CLI::App* app)
{}

bool
ifm3d::O3Cxxx::CheckCompatibility()
{
  if (!(Parent<MainCommand>()->GetDevice()->AmI(Device::DeviceFamily::O3C)))
    {
      return false;
    }

  return Command::CheckCompatibility();
}
