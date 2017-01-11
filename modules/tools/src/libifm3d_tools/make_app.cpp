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

#include <ifm3d/tools/make_app.h>
#include <exception>
#include <memory>
#include <string>
#include <boost/program_options.hpp>
#include <glog/logging.h>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/tools/reboot_app.h>
#include <ifm3d/camera.h>

namespace po = boost::program_options;

ifm3d::CmdLineApp::Ptr
ifm3d::make_app(int argc, const char **argv)
{
  po::options_description desc;
  desc.add_options()
    ("command", po::value<std::string>()->default_value("version"),
     "ifm3d Sub-command to execute");

  po::positional_options_description p;
  p.add("command", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
            options(desc).positional(p).allow_unregistered().run(), vm);
  po::notify(vm);


  std::string cmd = vm["command"].as<std::string>();
  if (cmd == "reboot")
    {
      return std::make_shared<ifm3d::RebootApp>(argc, argv);
    }
  else if (cmd == "version")
    {
      return std::make_shared<ifm3d::CmdLineApp>(argc, argv);
    }
  else
    {
      std::cerr << "Unknown sub-command: " << cmd << std::endl;
      throw ifm3d::error_t(IFM3D_SUBCOMMAND_ERROR);
    }
}
