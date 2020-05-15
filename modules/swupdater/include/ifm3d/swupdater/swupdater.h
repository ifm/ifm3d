/*
 * Copyright (C) 2019 ifm electronic, gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __IFM3D_SWUPDATER_SWUPDATER_H__
#define __IFM3D_SWUPDATER_SWUPDATER_H__

#include <memory>
#include <vector>
#include <ifm3d/camera/camera.h>

namespace ifm3d
{
  class SWUpdater
  {
  public:
    using Ptr = std::shared_ptr<SWUpdater>;

    /**
     * Signature for user callback to receive status information about firmware
     * flashing.
     *
     * The first parameter is a percentage (0.0-1.0) indicating the status of
     * uploading the file to the device.
     *
     * The second parameter is a status message from the camera during install.
     */
    using FlashStatusCb = std::function<void(float, const std::string&)>;

    /**
     * Ctor
     *
     * @param cam Camera object to manipulate
     *
     * @param cb Opitonal user-defined callback to handle status updates
     */
    SWUpdater(ifm3d::Camera::Ptr cam,
              const ifm3d::SWUpdater::FlashStatusCb& cb = {});

    virtual ~SWUpdater();

    // disable copy/move semantics
    SWUpdater(SWUpdater&&) = delete;
    SWUpdater& operator=(SWUpdater&&) = delete;
    SWUpdater(SWUpdater&) = delete;
    SWUpdater& operator=(const SWUpdater&) = delete;

    /**
     * Reboots the camera from productive to recovery. The function returns
     * immediately, but the reboot process takes some time. The function
     * `WaitForRecovery` may be used to poll for completion.
     *
     * @throw ifm3d::error_t on error
     */
    void RebootToRecovery();

    /**
     * Polls on status of the camera, waiting for it to present
     * in recovery mode. Should be used following a call to
     * RebootToRecovery().
     *
     * @param[in] timeout_millis Timeout in millis to wait for the recovery
     *                           system to become available.  If
     *                           `timeout_millis` is set to 0, this function
     *                           will block indefinitely. If `timeout_millis`
     *                           is set to -1, this function will check once
     *                           and return immediately.
     *
     * @return true if the recovery system became available w/in
     *              `timeout_millis`, false otherwise.
     *
     * @throw ifm3d::error_t on error
     */
    bool WaitForRecovery(long timeout_millis = 0);

    /**
     * Reboots the camera from recovery to productive. The function returns
     * immediately, but the reboot process takes some time. The function
     * `WaitForProductive` may be used to poll for completion.
     *
     * @throw ifm3d::error_t on error
     */
    void RebootToProductive();

    /**
     * Polls on status of the camera, waiting for it to present
     * in productive mode. Should be used following a call to
     * RebootToProductive().
     *
     * @param[in] timeout_millis Timeout in millis to wait for the device to
     *                           become available.  If `timeout_millis` is set
     *                           to 0, this function will block indefinitely.
     *                           If `timeout_millis` is set to -1, this
     *                           function will check once and return
     *                           immediately.
     *
     * @return true if the device became available w/in `timeout_millis`,
     *         false otherwise.
     *
     * @throw ifm3d::error_t on error
     */
    bool WaitForProductive(long timeout_millis = 0);

    /**
     * Uploads a firmware image to the camera's recovery system.
     * Assumes device has already been rebooted to recovery mode.
     *
     * @param[in] bytes The firmware image data to flash to the camera.
     *
     * @param[in] timeout_millis Timeout in millis to wait for the firmware
     *                           upload to complete. If `timeout_millis` is set
     *                           to 0, this function will block indefinitely.
     *
     *                           NOTE: Firmware uploading and flashing
     *                           typically takes several minutes. The blocking
     *                           version of the API (timeout_millis = 0) is
     *                           recommended in most cases. If a timeout is
     *                           truly required, it is recommended to use a
     *                           value of at least 300000 (5 minutes).
     *
     * @throw ifm3d::error_t on error
     */
    bool FlashFirmware(const std::vector<std::uint8_t>& bytes,
                       long timeout_millis = 0);

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

  }; // end: class SWUpdater

} // end: namespace ifm3d

#endif //__IFM3D_SWUPDATER_SWUPDATER_H__
