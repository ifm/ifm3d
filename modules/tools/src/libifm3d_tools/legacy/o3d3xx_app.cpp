/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include "ifm3d/tools/legacy/app_types_app.h"
#include "ifm3d/tools/common/config_set_app.h"
#include "ifm3d/tools/legacy/cp_app.h"
#include "ifm3d/tools/common/discover_app.h"
#include "ifm3d/tools/common/dump_app.h"
#include "ifm3d/tools/legacy/export_app.h"
#include "ifm3d/tools/common/fg/hz_app.h"
#include "ifm3d/tools/legacy/imager_types_app.h"
#include "ifm3d/tools/legacy/import_app.h"
#include "ifm3d/tools/common/fg/jitter_app.h"
#include "ifm3d/tools/legacy/ls_app.h"
#include "ifm3d/tools/legacy/passwd_app.h"
#include "ifm3d/tools/common/reboot_app.h"
#include "ifm3d/tools/common/reset_app.h"
#include "ifm3d/tools/legacy/rm_app.h"
#include "ifm3d/tools/common/swupdater/swupdate_app.h"
#include "ifm3d/device/device.h"
#include "ifm3d/tools/legacy/time_app.h"
#include "ifm3d/tools/common/trace_app.h"
#include "ifm3d/tools/command.hpp"
#include <ifm3d/tools/legacy/o3d3xx_app.h>
ifm3d::O3D3XX::~O3D3XX() = default;

CLI::App*
ifm3d::O3D3XX::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("o3d3xx", "Interact with the O3D3xx devices")
      ->require_subcommand(1);

  RegisterSubcommand<ifm3d::AppTypesApp>(command);
  RegisterSubcommand<ifm3d::ConfigSetApp>(command);
  RegisterSubcommand<ifm3d::CpApp>(command);
  RegisterSubcommand<ifm3d::DiscoverApp>(command);
  RegisterSubcommand<ifm3d::DumpApp>(command);
  RegisterSubcommand<ifm3d::ExportApp>(command);
#if defined(BUILD_MODULE_FRAMEGRABBER)
  RegisterSubcommand<ifm3d::HzApp>(command);
#endif
  RegisterSubcommand<ifm3d::ImagerApp>(command);
  RegisterSubcommand<ifm3d::ImportApp>(command);
#if defined(BUILD_MODULE_FRAMEGRABBER)
  RegisterSubcommand<ifm3d::JitterApp>(command);
#endif
  RegisterSubcommand<ifm3d::LsApp>(command);
  RegisterSubcommand<ifm3d::PasswordApp>(command);
  RegisterSubcommand<ifm3d::RebootApp>(command);
  RegisterSubcommand<ifm3d::ResetApp>(command);
  RegisterSubcommand<ifm3d::RmApp>(command);
#if defined(BUILD_MODULE_SWUPDATER)
  RegisterSubcommand<ifm3d::SWUpdateApp>(command,
                                         ifm3d::Device::swu_version::SWU_V1);
#endif
  RegisterSubcommand<ifm3d::TimeApp>(command);
  RegisterSubcommand<ifm3d::TraceApp>(command);

  return command;
}

void
ifm3d::O3D3XX::Execute(CLI::App* app)
{}

bool
ifm3d::O3D3XX::CheckCompatibility()
{
  if (!(Parent<MainCommand>()->GetDevice()->AmI(Device::device_family::O3D)))
    {
      return false;
    }

  return Command::CheckCompatibility();
}