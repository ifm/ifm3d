/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/import_app.h>
#include <cstdint>
#include <iostream>
#include <istream>
#include <exception>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/device.h>

ifm3d::ImportApp::ImportApp(int argc,
                            const char** argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->all_opts_.add_options(name)
    ("file",
     "Input file, defaults to `stdin' (good for reading off a pipeline)",
     cxxopts::value<std::string>()->default_value("-"))
    ("c,config",
     "Flag indicating the input is an entire sensor config (app otherwise)")
    ("g,global", "If `-c', import the global configuration")
    ("n,net", "If `-c', import the network configuration")
    ("a,app", "If `-c', import the application configuration");

  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::ImportApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  std::shared_ptr<std::istream> ifs;
  std::vector<std::uint8_t> bytes;

  std::string infile = (*this->vm_)["file"].as<std::string>();
  if (infile == "-")
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
      ifs.reset(new std::ifstream(infile, std::ios::in | std::ios::binary));
      if (!*ifs)
        {
          std::cerr << "Could not open file: " << infile << std::endl;
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
  if (!this->vm_->count("config"))
    {
      std::static_pointer_cast<ifm3d::LegacyDevice>(this->cam_)
        ->ImportIFMApp(bytes);
    }
  else
    {
      if (this->vm_->count("global"))
        {
          mask |= static_cast<std::uint16_t>(
            ifm3d::LegacyDevice::import_flags::GLOBAL);
        }

      if (this->vm_->count("net"))
        {
          mask |=
            static_cast<std::uint16_t>(ifm3d::LegacyDevice::import_flags::NET);
        }

      if (this->vm_->count("app"))
        {
          mask |= static_cast<std::uint16_t>(
            ifm3d::LegacyDevice::import_flags::APPS);
        }

      std::static_pointer_cast<ifm3d::LegacyDevice>(this->cam_)
        ->ImportIFMConfig(bytes, mask);
    }

  return 0;
}

bool
ifm3d::ImportApp::CheckCompatibility()
{
  return this->cam_->AmI(Device::device_family::O3D) ||
         this->cam_->AmI(Device::device_family::O3X);
}