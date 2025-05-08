/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/ovp8xx/ovp8xx_app.h>
#include <ifm3d/common/features.h>

ifm3d::OVP8xx::~OVP8xx() {}

CLI::App*
ifm3d::OVP8xx::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("ovp8xx",
                       "Interact with the OVP8xx video processing units")
      ->require_subcommand(1);
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
                                         ifm3d::Device::swu_version::SWU_V2);
#endif
  return command;
}

void
ifm3d::OVP8xx::Execute(CLI::App* app)
{}

bool
ifm3d::OVP8xx::CheckCompatibility()
{
  if (!(Parent<MainCommand>()->GetDevice()->AmI(Device::device_family::O3R)))
    {
      return false;
    }

  return Command::CheckCompatibility();
}