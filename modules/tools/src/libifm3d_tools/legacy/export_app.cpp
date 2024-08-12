/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/legacy/export_app.h>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <ifm3d/device.h>

ifm3d::ExportApp::~ExportApp() {}

void
ifm3d::ExportApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  std::vector<std::uint8_t> bytes;

  if (this->application_index <= 0)
    {
      bytes = std::static_pointer_cast<ifm3d::LegacyDevice>(device)
                ->ExportIFMConfig();
    }
  else
    {
      bytes =
        std::static_pointer_cast<ifm3d::LegacyDevice>(device)->ExportIFMApp(
          this->application_index);
    }

  if (this->output_file == "-")
    {
      std::cout.write(reinterpret_cast<char*>(bytes.data()), bytes.size());
    }
  else
    {
      std::ofstream(this->output_file, std::ios::binary)
        .write(reinterpret_cast<char*>(bytes.data()), bytes.size());
    }
}

CLI::App*
ifm3d::ExportApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("export",
                       "Export an application or whole "
                       "sensor configuration into a format compatible with "
                       "ifm Vision Assistant.")
      ->require_subcommand(0, 0);

  command
    ->add_option(
      "--file",
      this->output_file,
      "Output file, defaults to `stdout' (good for piping to other tools)")
    ->default_val("-");

  command
    ->add_option(
      "--index",
      this->application_index,
      "If provided, this specifies the index of an application to export")
    ->default_val(-1);

  return command;
}
