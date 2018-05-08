/*
 * Copyright (C) 2018 ifm syntron gmbh
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

#include <iostream>
#include <ifm3d/tools/trace_app.h>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>

ifm3d::TraceApp::TraceApp(int argc, const char **argv,
                    const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  this->local_opts_.add_options()
    ("limit,l", po::value<int>(),
     "Limit the amount of trace log messages printed. (default: all)");

  po::store(po::command_line_parser(argc, argv).
            options(this->local_opts_).allow_unregistered().run(), this->vm_);
  po::notify(this->vm_);
}

int ifm3d::TraceApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  auto limit = 0;
  if (this->vm_.count("limit"))
    {
      limit = std::max(1,this->vm_["limit"].as<int>());
    }

  std::vector<std::string> logs = this->cam_->TraceLogs(limit);

  for (auto& log : logs )
    {
      std::cout << log
                << std::endl
                << std::flush;
    }

  return 0;
}
