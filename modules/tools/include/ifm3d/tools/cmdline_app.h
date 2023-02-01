/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_CMDLINE_APP_H
#define IFM3D_TOOLS_CMDLINE_APP_H

#include <cstdint>
#include <memory>
#include <string>
#include <cxxopts.hpp>
#include <ifm3d/device/device.h>

namespace ifm3d
{
  /**
   * This class provides an interface for `ifm3d` command-line applications.
   *
   * Subclasses of this base class will implement a single sub-command
   * to the `ifm3d` command line utility. Each subclass is should 1) call
   * the base class ctor, 2) implement `Run`
   */
  class CmdLineApp
  {
  public:
    using Ptr = std::shared_ptr<CmdLineApp>;

    CmdLineApp(int argc,
               const char** argv,
               const std::string& name = "version",
               bool throwIfUnavailable = true);
    virtual ~CmdLineApp() = default;

    // copy and move semantics
    CmdLineApp(CmdLineApp&&) = delete;
    CmdLineApp& operator=(CmdLineApp&&) = delete;
    CmdLineApp(CmdLineApp&) = delete;
    CmdLineApp& operator=(const CmdLineApp&) = delete;

    /**
     * Sub-classes should override this method. This is the sub-class' hook to
     * implement its business logic (it is `main`).
     *
     * @return An error code suitable for returning to a shell
     */
    virtual int Run();

    /**
     * checks the subcommand and options compatibility with device.
     */
    bool CheckAppCompatibility();

    /**
     * Method to execute ifm3d commands
     */
    static int Execute(size_t argc, const char** argv);

  protected:
    cxxopts::Options all_opts_;
    std::unique_ptr<cxxopts::ParseResult> vm_;

    std::string ip_;
    std::uint16_t xmlrpc_port_;
    std::string password_;
    ifm3d::Device::Ptr cam_;

    virtual void _LocalHelp();

    /**
     * @brief Parse the argv and store values to variable map (vm)
     * @param argc : number of parameters
     * @param agrv : array of the command line options
     */
    void _Parse(int argc, const char** argv);

    /**
     * checks if the app is compatiable with device
     */
    virtual bool CheckCompatibility();

  }; // end: class CmdLineApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_CMDLINE_APP_H
