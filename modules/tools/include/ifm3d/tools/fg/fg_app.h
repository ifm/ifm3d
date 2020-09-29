// -*- c++ -*-
/*
 * Copyright 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_TOOLS_FG_APP_H__
#define __IFM3D_TOOLS_FG_APP_H__

#include <string>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/fg.h>

namespace ifm3d
{
  /**
   *  FgApp provides the interface for ifm3d::Framgrabber dependent command
   */
  class FgApp : public ifm3d::CmdLineApp
  {
  public:
    FgApp(int argc, const char** argv, const std::string& name = "");

    virtual ~FgApp() = default;

    // copy and move semantics
    FgApp(FgApp&&) = delete;
    FgApp& operator=(FgApp&&) = delete;
    FgApp(FgApp&) = delete;
    FgApp& operator=(const FgApp&) = delete;

  protected:
    unsigned short nat_port_;
    ifm3d::FrameGrabber::Ptr fg_;
  };

} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_FG_APP_H__
