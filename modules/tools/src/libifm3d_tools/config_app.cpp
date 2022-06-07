/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/config_app.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>

ifm3d::ConfigApp::ConfigApp(int argc,
                            const char** argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->all_opts_.add_options(name)
    ("file",
     "Input JSON configuration file (defaults to stdin)",
     cxxopts::value<std::string>()->default_value("-"));
  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::ConfigApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  std::string jstr;
  std::string infile = (*this->vm_)["file"].as<std::string>();

  if (infile == "-")
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

  this->cam_->FromJSONStr(jstr);
  return 0;
}
