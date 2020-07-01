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

#ifndef __IFM3D_TOOLS_MAKE_APP_H__
#define __IFM3D_TOOLS_MAKE_APP_H__

#include <iostream>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Factory function for instantiating the proper subcommand implementation
   * class based on the passed in command-line arguments.
   *
   * This function is simply looking for the first positional argument on the
   * command-line and using that as the key to instantiate the proper the
   * sub-command class.
   *
   * @param[in] argc Argument count from `main`
   * @param[in] argv Argument vector from `main`
   *
   * @return A smart pointer to an application implementing the passed in
   *         subcommand.
   */
  ifm3d::CmdLineApp::Ptr make_app(int argc, const char** argv);

} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_MAKE_APP_H__
