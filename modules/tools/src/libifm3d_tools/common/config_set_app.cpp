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
#include <ifm3d/device/util.h>

ifm3d::ConfigSetApp::~ConfigSetApp() {}

void
ifm3d::ConfigSetApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  std::string jstr;
  std::string infile = this->config_file;

  if (infile == "-")
    {
      std::ostringstream buff;

      if (ifm3d::IsStdinAvailable())
        {
          std::string line;
          while (std::getline(std::cin, line))
            {
              buff << line << std::endl;
            }
        }
      else
        {
          throw ifm3d::Error(IFM3D_NO_INPUT_PROVIDED);
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

  if (this->save)
    {
      std::static_pointer_cast<ifm3d::O3R>(device)->SaveInit();
    }
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
      command->add_flag("--save",
                        this->save,
                        "Save the new configuration as initial JSON");
    }

  return command;
}
