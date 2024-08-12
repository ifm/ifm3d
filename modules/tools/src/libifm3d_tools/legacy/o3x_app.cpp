/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/legacy/o3x_app.h>
#include <iostream>

ifm3d::O3X1XX_O3X2XX::~O3X1XX_O3X2XX() {}

CLI::App*
ifm3d::O3X1XX_O3X2XX::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("O3X", "Interact with O3X devices")
      ->require_subcommand(1);

  RegisterSubcommand<ifm3d::AppTypesApp>(command);
  RegisterSubcommand<ifm3d::ConfigSetApp>(command);
  RegisterSubcommand<ifm3d::CpApp>(command);
  RegisterSubcommand<ifm3d::DiscoverApp>(command);
  RegisterSubcommand<ifm3d::DumpApp>(command);
  RegisterSubcommand<ifm3d::ExportApp>(command);
  RegisterSubcommand<ifm3d::HzApp>(command);
  RegisterSubcommand<ifm3d::ImagerApp>(command);
  RegisterSubcommand<ifm3d::ImportApp>(command);
  RegisterSubcommand<ifm3d::JitterApp>(command);
  RegisterSubcommand<ifm3d::LsApp>(command);
  RegisterSubcommand<ifm3d::PasswordApp>(command);
  RegisterSubcommand<ifm3d::RebootApp>(command);
  RegisterSubcommand<ifm3d::ResetApp>(command);
  RegisterSubcommand<ifm3d::RmApp>(command);
  RegisterSubcommand<ifm3d::SWUpdateApp>(command);
  RegisterSubcommand<ifm3d::TimeApp>(command);
  RegisterSubcommand<ifm3d::TraceApp>(command);

  return command;
}

void
ifm3d::O3X1XX_O3X2XX::Execute(CLI::App* app)
{}

bool
ifm3d::O3X1XX_O3X2XX::CheckCompatibility()
{
  if (!(Parent<MainCommand>()->GetDevice()->AmI(Device::device_family::O3X)))
    {
      return false;
    }

  return Command::CheckCompatibility();
}