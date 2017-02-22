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

#include <ifm3d/tools/config_app.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <boost/program_options.hpp>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>
#include <ifm3d/camera/err.h>

namespace po = boost::program_options;

ifm3d::ConfigApp::ConfigApp(int argc, const char **argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  this->local_opts_.add_options()
    ("file", po::value<std::string>()->default_value("-"),
     "Input JSON configuration file (defaults to stdin)");

  po::store(po::command_line_parser(argc, argv).
            options(this->local_opts_).allow_unregistered().run(), this->vm_);
  po::notify(this->vm_);
}

int ifm3d::ConfigApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  std::string jstr;
  std::string infile = this->vm_["file"].as<std::string>();

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
      if (! ifs)
        {
          std::cerr << "Could not parse file: " << infile << std::endl;
          throw ifm3d::error_t(IFM3D_IO_ERROR);
        }

      jstr.assign((std::istreambuf_iterator<char>(ifs)),
                  (std::istreambuf_iterator<char>()));
    }

  this->cam_->FromJSONStr(jstr);
  return 0;
}
