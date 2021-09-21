/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_CAMERA_CAMERA_O3R_H
#define IFM3D_CAMERA_CAMERA_O3R_H

#include <ifm3d/camera/camera_base.h>

namespace ifm3d
{
  /**
   * Camera specialization for O3R
   */
  class O3RCamera : public CameraBase
  {
  public:
    using Ptr = std::shared_ptr<O3RCamera>;
    O3RCamera(const std::string& ip = ifm3d::DEFAULT_IP,
              const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT);

    virtual ~O3RCamera();
    O3RCamera(O3RCamera&&) = delete;
    O3RCamera& operator=(O3RCamera&&) = delete;
    O3RCamera(O3RCamera&) = delete;
    O3RCamera& operator=(O3RCamera&) = delete;

    /**
     * Sets the camera configuration back to the state in which it shipped from
     * the ifm factory.
     *
     * @param[in] keepNetworkSettings a bool indicating wether to keep the
     * current network settings
     */
    virtual void FactoryReset(bool keepNetworkSettings);

    /**
     * Return the current JSON schema configuration
     *
     * @return The current JSON schema configuration
     */
    json Schema();

    void Reboot(const boot_mode& mode =
                  ifm3d::CameraBase::boot_mode::PRODUCTIVE) override;
    device_family WhoAmI() override;
    ifm3d::CameraBase::swu_version SwUpdateVersion() override;

    json ToJSON() override;
    void FromJSON(const json& j) override;

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
  }; // end: class O3RCamera
}
#endif // IFM3D_CAMERA_CAMERA_O3R_H