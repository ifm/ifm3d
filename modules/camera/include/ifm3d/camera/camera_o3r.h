/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_CAMERA_CAMERA_O3R_H
#define IFM3D_CAMERA_CAMERA_O3R_H

#include <ifm3d/camera/camera_base.h>

namespace ifm3d
{
  struct PortInfo
  {
    std::string port;
    uint16_t pcic_port;
    std::string type;
  };

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
    json GetSchema();

    /**
     * Returns the configuration formatted as JSON based on a path.
     * If the path is empty, returns the whole configuration.
     *
     * @param[in] path A List of JSON path fragments to retrieve the
     * information for
     *
     * @return The JSON configuration for the list of object path fragments
     */
    json Get(
      const std::vector<std::string>& path = std::vector<std::string>());

    /**
     * Returns a part of the configuration formatted as JSON based on a
     * JSON pointer.
     *
     * @param[in] ptr A JSON pointer to retrieve the information for
     *
     * @return The partial JSON configuration for the given JSON pointer
     */
    json ResolveConfig(const json::json_pointer& ptr);

    /**
     * Overwrites parts of the temporary JSON configuration which is achieved
     * by merging the provided JSON fragment with the current temporary JSON.
     *
     * @param[in] j The new temporay JSON configuration of the device.
     */
    void Set(const json& j);

    /**
     * Return the initial JSON configuration.
     *
     * @return The initial JSON configuration
     */
    json GetInit();

    /**
     * Save to current temporary JSON configuration as initial JSON
     * configuration
     */
    void SaveInit();

    /**
     * Returns the init status of the device
     *
     * @return The init status of the device
     */
    std::string GetInitStatus();

    /**
     * Release the lock from the Device
     *
     * @param[in] password the password used to unlock the device
     */
    void Lock(const std::string& password);

    /**
     * Locks the device until it is unlocked.
     * If the device is unlocked and an empty password is provided the password
     * protection is removed.
     *
     * @param[in] password the password used to lock the device
     */
    void Unlock(const std::string& password);

    /**
     * Returns a list containing information about all connected physical ports
     *
     * @return the list of ports
     */
    std::vector<PortInfo> Ports();

    /**
     * Returns information about a given physical port
     *
     * @param[in] port the port for which to get the information
     *
     * @return the port information
     */
    PortInfo Port(const std::string& port);

    void Reboot(const boot_mode& mode =
                  ifm3d::CameraBase::boot_mode::PRODUCTIVE) override;
    device_family WhoAmI() override;
    ifm3d::CameraBase::swu_version SwUpdateVersion() override;

    /**
     * @copydoc CameraBase::ToJSON()
     * Equivalent to the @ref Get() method
     */
    json ToJSON() override;
    /**
     * @copydoc CameraBase::FromJSON()
     * Equivalent to @ref Set() followed by @ref SaveInit()
     */
    void FromJSON(const json& j) override;

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
  }; // end: class O3RCamera
}
#endif // IFM3D_CAMERA_CAMERA_O3R_H