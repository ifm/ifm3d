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

#include <ifm3d/tools/app_types_app.h>
#include <iostream>
#include <string>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>

ifm3d::AppTypesApp::AppTypesApp(int argc, const char **argv,
                                const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{ }

int ifm3d::AppTypesApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  json app_types(this->cam_->ApplicationTypes());
  std::cout << app_types.dump(2) << std::endl;

  return 0;
}
