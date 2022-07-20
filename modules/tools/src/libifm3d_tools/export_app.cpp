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
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/device.h>

ifm3d::ExportApp::ExportApp(int argc,
                            const char** argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->all_opts_.add_options(name)
    ("file", "Output file, defaults to `stdout' (good for piping to other tools)",
     cxxopts::value<std::string>()->default_value("-"))
    ("index",
     "If provided, this specifies the index of an application to export",
     cxxopts::value<int>()->default_value("-1"));
  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::ExportApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  int idx = (*this->vm_)["index"].as<int>();
  std::vector<std::uint8_t> bytes;

  if (idx <= 0)
    {
      bytes = std::static_pointer_cast<ifm3d::LegacyDevice>(this->cam_)
                ->ExportIFMConfig();
    }
  else
    {
      bytes = std::static_pointer_cast<ifm3d::LegacyDevice>(this->cam_)
                ->ExportIFMApp(idx);
    }

  std::string outfile = (*this->vm_)["file"].as<std::string>();
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

bool
ifm3d::ExportApp::CheckCompatibility()
{
  return this->cam_->AmI(Device::device_family::O3D) ||
         this->cam_->AmI(Device::device_family::O3X);
}