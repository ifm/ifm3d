// -*- c++ -*-
/*
 * Copyright (C) 2018 Christian Ege
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

#ifndef __IFM3D_TOOLS_SWUPDATE_APP_H__
#define __IFM3D_TOOLS_SWUPDATE_APP_H__

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
    SwupdateApp(int argc, const char **argv,
                const std::string& name = "swupdate");
    int Run();

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

#endif // __IFM3D_TOOLS_SWUPDATE_APP_H__
