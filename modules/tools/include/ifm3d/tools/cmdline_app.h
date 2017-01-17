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

#ifndef __IFM3D_TOOLS_CMDLINE_APP_H__
#define __IFM3D_TOOLS_CMDLINE_APP_H__

#include <cstdint>
#include <memory>
#include <string>
#include <boost/program_options.hpp>
#include <ifm3d/camera/camera.h>

namespace po = boost::program_options;

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

    CmdLineApp(int argc, const char **argv,
               const std::string& name = "version");
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

  protected:
    po::variables_map vm_;
    po::options_description global_opts_;
    po::options_description local_opts_;

    std::string ip_;
    std::uint16_t xmlrpc_port_;
    std::string password_;
    ifm3d::Camera::Ptr cam_;

    virtual void _LocalHelp();

  }; // end: class CmdLineApp

} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_CMDLINE_APP_H__
