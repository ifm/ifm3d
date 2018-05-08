// -*- c++ -*-
/*
 * Copyright (C) 2018 ifm electronics, gmbh
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

#ifndef __IFM3D_TOOLS_PASSWD_APP_H__
#define __IFM3D_TOOLS_PASSWD_APP_H__

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `reboot` subcommand to the `ifm3d`
   * command-line utility.
   */
  class PasswdApp : public ifm3d::CmdLineApp
  {
  public:
	  PasswdApp(int argc, const char **argv, const std::string& name = "password");
    int Run();
  }; // end: class Passwd
} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_PASSWD_APP_H__
