/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_SWUPDATER_SWUPDATER_H
#define IFM3D_SWUPDATER_SWUPDATER_H

#include <memory>
#include <vector>
#include <ifm3d/device/legacy_device.h>
#include <ifm3d/swupdater/module_swupdater.h>

namespace ifm3d
{
  /* const for the swupdate recovery port value */
  extern IFM3D_EXPORT const std::uint16_t SWUPDATER_RECOVERY_PORT;

  /** @ingroup SWUpdater */
  class IFM3D_EXPORT SWUpdater
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
     * The second parameter is a status message from the device during install.
     */
    using FlashStatusCb = std::function<void(float, const std::string&)>;

    /**
     * Ctor
     *
     * @param cam Device object to manipulate
     *
     * @param cb Optional user-defined callback to handle status updates
     *
     * @param swupdate_recovery_port swupate recovery port for the device
     */
    SWUpdater(ifm3d::Device::Ptr cam,
              const ifm3d::SWUpdater::FlashStatusCb& cb = {},
              const std::uint16_t swupdate_recovery_port =
                ifm3d::SWUPDATER_RECOVERY_PORT);

    virtual ~SWUpdater();

    // disable copy/move semantics
    SWUpdater(SWUpdater&&) = delete;
    SWUpdater& operator=(SWUpdater&&) = delete;
    SWUpdater(SWUpdater&) = delete;
    SWUpdater& operator=(const SWUpdater&) = delete;

    /**
     * Reboots the device from productive to recovery. The function returns
     * immediately, but the reboot process takes some time. The function
     * `WaitForRecovery` may be used to poll for completion.
     *
     * @throw ifm3d::error_t on error
     */
    void RebootToRecovery();

    /**
     * Polls on status of the device, waiting for it to present
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
     * Reboots the device from recovery to productive. The function returns
     * immediately, but the reboot process takes some time. The function
     * `WaitForProductive` may be used to poll for completion.
     *
     * @throw ifm3d::error_t on error
     */
    void RebootToProductive();

    /**
     * Polls on status of the device, waiting for it to present
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
     * Uploads a firmware image to the device's recovery system.
     * Assumes device has already been rebooted to recovery mode.
     *
     * @param[in] swu_file The firmware image file to flash to the device.
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
    bool FlashFirmware(const std::string& swu_file, long timeout_millis = 0);

    class IFM3D_NO_EXPORT Impl;

  private:
    std::unique_ptr<Impl> pImpl;

  }; // end: class SWUpdater

} // end: namespace ifm3d

#endif // IFM3D_SWUPDATER_SWUPDATER_H
