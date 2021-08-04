// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_MAKE_APP_H
#define IFM3D_TOOLS_MAKE_APP_H

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

#endif // IFM3D_TOOLS_MAKE_APP_H
