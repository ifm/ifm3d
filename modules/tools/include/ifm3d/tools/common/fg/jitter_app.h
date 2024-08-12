// -*- c++ -*-
/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_TOOLS_JITTER_APP_H
#define IFM3D_TOOLS_JITTER_APP_H

#include <string>
#include <CLI/CLI.hpp>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <ifm3d/tools/tools_export.h>
#include <ifm3d/fg.h>

namespace ifm3d
{
  extern IFM3D_TOOLS_EXPORT const int FG_JITTER_TIMEOUT;
  /**
   * Concrete implementation of the `jitter` subcommand to the `ifm3d`
   * command-line utility.
   */
  class JitterApp : public Command
  {
  public:
    ~JitterApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    void capture_frames(ifm3d::FrameGrabber::Ptr fg,
                        std::vector<float>& results);

    unsigned short pcic_port{(unsigned short)ifm3d::DEFAULT_PCIC_PORT};
    int nframes{100};
    std::string output_file{"-"};

  }; // end: class JitterApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_JITTER_APP_H
