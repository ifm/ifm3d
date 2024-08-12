/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/legacy/import_app.h>
#include <cstdint>
#include <iostream>
#include <istream>
#include <exception>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <ifm3d/device.h>

ifm3d::ImportApp::~ImportApp() {}

void
ifm3d::ImportApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  std::shared_ptr<std::istream> ifs;
  std::vector<std::uint8_t> bytes;

  if (this->input_file == "-")
    {
      ifs.reset(&std::cin, [](...) {});

      char b;
      while (ifs->get(b))
        {
          bytes.push_back(*(reinterpret_cast<std::uint8_t*>(&b)));
        }
    }
  else
    {
      ifs.reset(
        new std::ifstream(this->input_file, std::ios::in | std::ios::binary));
      if (!*ifs)
        {
          std::cerr << "Could not open file: " << this->input_file
                    << std::endl;
          throw ifm3d::Error(IFM3D_IO_ERROR);
        }

      ifs->unsetf(std::ios::skipws);
      std::streampos file_size;
      ifs->seekg(0, std::ios::end);
      file_size = ifs->tellg();
      ifs->seekg(0, std::ios::beg);

      bytes.reserve(file_size);
      bytes.insert(bytes.begin(),
                   std::istream_iterator<std::uint8_t>(*ifs),
                   std::istream_iterator<std::uint8_t>());
    }

  std::uint16_t mask = 0x0;
  if (!config->count())
    {
      std::static_pointer_cast<ifm3d::LegacyDevice>(device)->ImportIFMApp(
        bytes);
    }
  else
    {
      if (global_config)
        {
          mask |= static_cast<std::uint16_t>(
            ifm3d::LegacyDevice::import_flags::GLOBAL);
        }

      if (network_config)
        {
          mask |=
            static_cast<std::uint16_t>(ifm3d::LegacyDevice::import_flags::NET);
        }

      if (app_config)
        {
          mask |= static_cast<std::uint16_t>(
            ifm3d::LegacyDevice::import_flags::APPS);
        }

      std::static_pointer_cast<ifm3d::LegacyDevice>(device)->ImportIFMConfig(
        bytes,
        mask);
    }
}

CLI::App*
ifm3d::ImportApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("import",
                       "Import an application or whole sensor configuration "
                       "that is compatible with "
                       "ifm Vision Assistant's export format.")
      ->require_subcommand(0, 0);

  command
    ->add_option(
      "--file",
      this->input_file,
      "Input file, defaults to `stdin' (good for reading off a pipeline)")
    ->default_val("-");

  config =
    command
      ->add_flag(
        "-c,--config",
        "Flag indicating the input is an entire sensor config (app otherwise)")
      ->default_str("flag");

  command
    ->add_flag("-g,--global",
               this->global_config,
               "If `-c', import the global configuration")
    ->needs(config)
    ->default_str("flag");

  command
    ->add_flag("-n,--net",
               this->network_config,
               "If `-c', import the network configuration")
    ->needs(config)
    ->default_str("flag");

  command
    ->add_flag("-a,--app",
               this->app_config,
               "If `-c', import the application configuration")
    ->needs(config)
    ->default_str("flag");

  return command;
}
