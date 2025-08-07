/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <cstdint>
#include <fstream>
#include <ifm3d/device/legacy_device.h>
#include <ifm3d/tools/legacy/export_application_app.h>
#include <ifm3d/tools/legacy/o3d3xx_app.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

ifm3d::ExportApplicationApp::~ExportApplicationApp() = default;

void
ifm3d::ExportApplicationApp::Execute(CLI::App* /*app*/)
{
  auto device = Parent<MainCommand>()->GetDevice();

  std::vector<std::uint8_t> bytes;

  if (Parent<ifm3d::O3D3XX>())
    {
      bytes =
        std::static_pointer_cast<ifm3d::LegacyDevice>(device)->ExportIFMApp(
          this->application_index);
    }
  else
    {
      bytes =
        std::static_pointer_cast<ifm3d::LegacyDevice>(device)->ExportIFMApp(1);
    }

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
ifm3d::ExportApplicationApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("app",
                       "Export an application that is compatible with ifm "
                       "Vision Assistant's export format (*.o3d3xxapp).")
      ->require_subcommand(0, 0);

  if (Parent<ifm3d::O3D3XX>())
    {
      command
        ->add_option("--index",
                     this->application_index,
                     "The index of an application to export")
        ->default_val(0)
        ->required();
    }

  command
    ->add_option(
      "--file",
      this->output_file,
      "Output file, defaults to `stdout' (good for piping to other tools)")
    ->default_val("-");

  return command;
}
