/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/common/config_set_app.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>

ifm3d::ConfigSetApp::~ConfigSetApp() {}

void
ifm3d::ConfigSetApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  std::string jstr;
  std::string infile = this->config_file;

  if (!this->path.empty())
    {
      jstr.assign(this->path);
    }
  else if (infile == "-")
    {
      std::string line;
      std::ostringstream buff;
      while (std::getline(std::cin, line))
        {
          buff << line << std::endl;
        }

      jstr.assign(buff.str());
    }
  else
    {
      std::ifstream ifs(infile, std::ios::in);
      if (!ifs)
        {
          std::cerr << "Could not parse file: " << infile << std::endl;
          throw ifm3d::Error(IFM3D_IO_ERROR);
        }

      jstr.assign((std::istreambuf_iterator<char>(ifs)),
                  (std::istreambuf_iterator<char>()));
    }
  device->FromJSONStr(jstr);
}

CLI::App*
ifm3d::ConfigSetApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("config",
                       "Configure sensor settings from a JSON description of "
                       "the desired sensor state. "
                       "See also 'dump'.")
      ->require_subcommand(0, 0);
  command
    ->add_option("--file",
                 this->config_file,
                 "Input JSON configuration file (defaults to stdin)")
    ->option_text("TEXT")
    ->default_str("-");

  if (Parent<ifm3d::OVP8xx>())
    {
      command->add_option(
        "--path",
        this->path,
        "Limit which part of the current configuration should be saved as "
        "initial JSON. ");
    }

  return command;
}
