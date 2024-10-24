/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/legacy/import_application_app.h>
#include <cstdint>
#include <iostream>
#include <istream>
#include <exception>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <ifm3d/device.h>
#include <ifm3d/device/util.h>

ifm3d::ImportApplicationApp::~ImportApplicationApp() {}

void
ifm3d::ImportApplicationApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  std::shared_ptr<std::istream> ifs;
  std::vector<std::uint8_t> bytes;

  if (this->input_file == "-")
    {
      if (ifm3d::IsStdinAvailable())
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
          throw ifm3d::Error(IFM3D_NO_INPUT_PROVIDED);
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
  std::static_pointer_cast<ifm3d::LegacyDevice>(device)->ImportIFMApp(bytes);
}

CLI::App*
ifm3d::ImportApplicationApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("app",
                       "Import an application that is compatible with ifm "
                       "Vision Assistant's export format (*.o3d3xxapp).")
      ->require_subcommand(0, 0);

  command
    ->add_option(
      "--file",
      this->input_file,
      "Input file, defaults to `stdin' (good for reading off a pipeline)")
    ->default_val("-");

  return command;
}
