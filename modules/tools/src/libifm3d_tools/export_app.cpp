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

ifm3d::ExportApp::ExportApp(int argc, const char **argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  this->local_opts_.add_options()
    ("file", po::value<std::string>()->default_value("-"),
     "Output file, defaults to `stdout' (good for piping to other tools)")
    ("index",
     po::value<int>()->default_value(-1),
     "If provided, this specifies the index of an application to export");

  po::store(po::command_line_parser(argc, argv).
            options(this->local_opts_).allow_unregistered().run(), this->vm_);
  po::notify(this->vm_);
}

int ifm3d::ExportApp::Run()
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
      std::cout.write(reinterpret_cast<char *>(bytes.data()), bytes.size());
    }
  else
    {
      std::ofstream(outfile, std::ios::binary).
        write(reinterpret_cast<char *>(bytes.data()), bytes.size());
    }

  return 0;
}
