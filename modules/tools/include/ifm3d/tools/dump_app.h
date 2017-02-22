// -*- c++ -*-
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

#ifndef __IFM3D_TOOLS_DUMP_APP_H__
#define __IFM3D_TOOLS_DUMP_APP_H__

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `dump` subcommand to the `ifm3d`
   * command-line utility.
   *
   * `dump` serializes the camera state to JSON
   */
  class DumpApp : public ifm3d::CmdLineApp
  {
  public:
    DumpApp(int argc, const char **argv, const std::string& name = "dump");
    int Run();
  }; // end: class DumpApp

} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_DUMP_APP_H__
