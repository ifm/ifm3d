/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include "ifm3d/device/legacy_device.h"
#include <ifm3d/tools/legacy/export_device_app.h>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

ifm3d::ExportDeviceApp::~ExportDeviceApp() = default;

void
ifm3d::ExportDeviceApp::Execute(CLI::App* /*app*/)
{
  auto device = Parent<MainCommand>()->GetDevice();

  std::vector<std::uint8_t> bytes;

  bytes =
    std::static_pointer_cast<ifm3d::LegacyDevice>(device)->ExportIFMConfig();

  if (this->output_file == "-")
    {
      std::cout.write(reinterpret_cast<char*>(bytes.data()),
                      static_cast<std::streamsize>(bytes.size()));
    }
  else
    {
      std::ofstream(this->output_file, std::ios::binary)
        .write(reinterpret_cast<char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
    }
}

CLI::App*
ifm3d::ExportDeviceApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("device",
                       "Export a device configuration that is compatible with "
                       "ifm Vision Assistant's export format. (*.o3d3xxcfg)")
      ->require_subcommand(0, 0);

  command
    ->add_option(
      "--file",
      this->output_file,
      "Output file, defaults to `stdout' (good for piping to other tools)")
    ->default_val("-");

  return command;
}
