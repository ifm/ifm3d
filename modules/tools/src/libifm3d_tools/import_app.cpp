/*
 * Copyright (C) 2017 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
#include <boost/program_options.hpp>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>
#include <ifm3d/camera/err.h>
#include <glog/logging.h>

namespace po = boost::program_options;

ifm3d::ImportApp::ImportApp(int argc, const char **argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  this->local_opts_.add_options()
    ("file", po::value<std::string>()->default_value("-"),
     "Input file, defaults to `stdin' (good for reading off a pipeline)")
    ("config,c",
     "Flag indicating the input is an entire sensor config (app otherwise)")
    ("global,g", "If `-c', import the global configuration")
    ("net,n", "If `-c', import the network configuration")
    ("app,a", "If `-c', import the application configuration");


  po::store(po::command_line_parser(argc, argv).
            options(this->local_opts_).allow_unregistered().run(), this->vm_);
  po::notify(this->vm_);
}

int ifm3d::ImportApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  std::shared_ptr<std::istream> ifs;
  std::vector<std::uint8_t> bytes;

  std::string infile = this->vm_["file"].as<std::string>();
  if (infile == "-")
    {
      ifs.reset(&std::cin, [](...){});

      char b;
      while (ifs->get(b))
        {
          bytes.push_back(*(reinterpret_cast<std::uint8_t*>(&b)));
        }
    }
  else
    {
      ifs.reset(new std::ifstream(infile, std::ios::in|std::ios::binary));
      if (! *ifs)
        {
          std::cerr << "Could not open file: " << infile << std::endl;
          throw ifm3d::error_t(IFM3D_IO_ERROR);
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
  if(! this->vm_.count("config"))
    {
      this->cam_->ImportIFMApp(bytes);
    }
  else
    {
      if (this->vm_.count("global"))
        {
          mask |=
            static_cast<std::uint16_t>(ifm3d::Camera::import_flags::GLOBAL);
        }

      if (this->vm_.count("net"))
        {
          mask |= static_cast<std::uint16_t>(ifm3d::Camera::import_flags::NET);
        }

      if (this->vm_.count("app"))
        {
          mask |= static_cast<std::uint16_t>(ifm3d::Camera::import_flags::APPS);
        }

      this->cam_->ImportIFMConfig(bytes, mask);
    }

  return 0;
}
