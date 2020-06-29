/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/export_app.h>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>

namespace po = boost::program_options;

ifm3d::ExportApp::ExportApp(int argc,
                            const char** argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->local_opts_.add_options()
    ("file", po::value<std::string>()->default_value("-"),
     "Output file, defaults to `stdout' (good for piping to other tools)")
    ("index",
     po::value<int>()->default_value(-1),
     "If provided, this specifies the index of an application to export");
  // clang-format on

  po::store(po::command_line_parser(argc, argv)
              .options(this->local_opts_)
              .allow_unregistered()
              .run(),
            this->vm_);
  po::notify(this->vm_);
}

int
ifm3d::ExportApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  int idx = this->vm_["index"].as<int>();
  std::vector<std::uint8_t> bytes;

  if (idx <= 0)
    {
      bytes = this->cam_->ExportIFMConfig();
    }
  else
    {
      bytes = this->cam_->ExportIFMApp(idx);
    }

  std::string outfile = this->vm_["file"].as<std::string>();
  if (outfile == "-")
    {
      std::cout.write(reinterpret_cast<char*>(bytes.data()), bytes.size());
    }
  else
    {
      std::ofstream(outfile, std::ios::binary)
        .write(reinterpret_cast<char*>(bytes.data()), bytes.size());
    }

  return 0;
}
