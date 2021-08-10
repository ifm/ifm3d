// -*- c++ -*-
/*
 * Copyright (C) 2018 Christian Ege
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SWUPDATE_APP_H
#define IFM3D_TOOLS_SWUPDATE_APP_H

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementatoin of the `swupdate` subcommand to the `ifm3d`
   * command-line utility.
   */
  class SwupdateApp : public ifm3d::CmdLineApp
  {
  public:
    SwupdateApp(int argc,
                const char** argv,
                const std::string& name = "swupdate");
    int Run() override;

  private:
    /**
     * Check if the device is in recovery mode
     */
    void checkRecovery();

    /**
     * Upload the update to the device
     */
    void uploadData(std::shared_ptr<std::istream> data, size_t filesize);

    /**
     * Wait until the update process finishes
     */
    void waitForUpdateFinish(int status);

    /**
     * Reboot the device to production mode
     */
    void reboot();

    /**
     * Checks wether the camera is in recovery mode
     * returns 0 for recovery mode and
     *    -1 for production mode
     *    -2 for device not connected
     */
    int checkRecoveryMode();

    /**
     * Reboot device from recovery mode to production mode
     */
    int rebootToProductiveMode();

    /**
     * upload firmware file to the device
     */
    int uploadFiletoDevice();

  }; // end: class SwupdateApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_SWUPDATE_APP_H
