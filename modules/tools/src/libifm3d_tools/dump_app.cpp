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

#include <ifm3d/tools/dump_app.h>
#include <iostream>
#include <string>
#include <ifm3d/tools/cmdline_app.h>

ifm3d::DumpApp::DumpApp(int argc, const char **argv,
                        const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{ }

int ifm3d::DumpApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  std::cout << this->cam_->ToJSONStr() << std::endl;

  return 0;
}
