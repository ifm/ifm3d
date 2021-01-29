// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_TOOLS_TIME_APP_H__
#define __IFM3D_TOOLS_TIME_APP_H__

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementatoin of the `time` subcommand to the `ifm3d`
   * command-line utility.
   */
  class TimeApp : public ifm3d::CmdLineApp
  {
  public:
    TimeApp(int argc, const char** argv, const std::string& name = "time");
    int Run();

  }; // end: class TimeApp

} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_TIME_APP_H__
