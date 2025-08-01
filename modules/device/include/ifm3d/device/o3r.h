/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_O3R_H
#define IFM3D_DEVICE_O3R_H

#include <ifm3d/common/features.h>
#include <ifm3d/device/device.h>
#ifdef BUILD_MODULE_CRYPTO
#  include <ifm3d/crypto/crypto.h>
#endif

namespace ifm3d
{
  static const int NET_WAIT_O3R_SET =
    std::getenv("IFM3D_NET_WAIT_O3R_SET") == nullptr ?
      15000 :
      std::stoi(std::getenv("IFM3D_NET_WAIT_O3R_SET"));

  /** @ingroup Device */
  struct IFM3D_EXPORT PortInfo
  {
    std::string port;
    uint16_t pcic_port;
    std::string type;
  };

#ifdef BUILD_MODULE_CRYPTO
  class O3RSealedBox;
#endif

  /** @ingroup Device
   *
   * Device specialization for O3R
   */
  class IFM3D_EXPORT O3R : public Device
  {
  public:
    using Ptr = std::shared_ptr<O3R>;
    O3R(const std::string& ip = ifm3d::DEFAULT_IP,
        const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT);

    virtual ~O3R();
    O3R(O3R&&) = delete;
    O3R& operator=(O3R&&) = delete;
    O3R(O3R&) = delete;
    O3R& operator=(O3R&) = delete;

    /**
     * Sets the device configuration back to the state in which it shipped from
     * the ifm factory.
     *
     * @param[in] keepNetworkSettings a bool indicating wether to keep the
     * current network settings
     */
    virtual void FactoryReset(bool keep_network_settings);

    /**
     * Return the current JSON schema configuration
     *
     * @return The current JSON schema configuration
     */
    json GetSchema();

    /**
     * Returns the configuration formatted as JSON based on a path.
     * If the path is empty, returns the whole configuration.
     * This function is blocking for firmware versions above 1.1.0.
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
     * This function is blocking for firmware versions above 1.1.0.
     *
     * @param[in] j The new temporay JSON configuration of the device.
     */
    void Set(const json& j);

    /**
     *  Removes an object from the JSON. The scope of this method is limited to
     *  the following regular expressions
     *
     *   * ^\/applications\/instances\/app\d+$
     *   * ^\/device\/log\/components\/[a-zA-Z0-9\-_]+$
     *
     * @param[in] jsonPointer A JSON Pointer to the object to be removed.
     */
    void Remove(const std::string& json_pointer);

    /**
     * Sets the default value of an object inside the JSON. The object is
     * addressed by a JSON Pointer. The object is reset to the values
     * defined in the JSON schema. Note that this does not reset the init
     * configuration, nor the parameters marked as "sticky".
     *
     * @param[in] jsonPointer A JSON Pointer to the object to be set to
     * default.
     */
    void Reset(const std::string& json_pointer);

    /**
     * Return the initial JSON configuration.
     *
     * @return The initial JSON configuration
     */
    json GetInit();

    /**
     * Save to current temporary JSON configuration as initial JSON
     * configuration, so it will be applied with the next transition to the
     * INIT state (system boot up)
     *
     * @param[in] pointers A List of JSON pointers specifying which parts of
     * the configuration should be saved as initial JSON. If no list is
     * provided the whole config will be saved
     */
    void SaveInit(const std::vector<std::string>& pointers = {});

    /**
     * @private
     *
     * Returns the init status of the device
     *
     * @return The init status of the device
     */
    std::string GetInitStatus();

    /**
     * @private
     *
     * Locks the device until it is unlocked.
     * If the device is unlocked and an empty password is provided the password
     * protection is removed.
     *
     * @param[in] password the password used to unlock the device
     */
    void Lock(const std::string& password);

    /**
     * @private
     *
     * Release the lock from the Device
     *
     * @param[in] password the password used to lock the device
     */
    void Unlock(const std::string& password);

    /**
     * Returns a list containing information about all connected physical and
     * application ports
     *
     * @return the list of ports
     */
    std::vector<PortInfo> Ports();

    /**
     * Returns information about a given physical or application port
     *
     * @param[in] port the port for which to get the information
     *
     * @return the port information
     */
    PortInfo Port(const std::string& port);

    /**
     * @brief Returns the content of the diagnostic memory formatted in JSON
     *
     * @return The JSON containing all known diagnostic events
     */
    json GetDiagnostic();

    /**
     * @brief Returns the JSON schema for the filter expression provided to the
     * getFiltered() method
     *
     * @return The JSON schema
     */
    json GetDiagnosticFilterSchema();

    /**
     * @brief Returns the content of the diagnostic memory formatted in JSON
     * and filtered according to the JSON filter expression
     *
     * @param filter A filter expression in JSON format
     * @return json
     */
    json GetDiagnosticFiltered(json filter);

    void Reboot(
      const boot_mode& mode = ifm3d::Device::boot_mode::PRODUCTIVE) override;

    /**
     * Reboot the device into Recovery Mode
     */
    void RebootToRecovery();

    device_family WhoAmI() override;
    ifm3d::Device::swu_version SwUpdateVersion() override;

    /**
     * @copydoc Device::ToJSON()
     * Equivalent to the @ref Get() method
     */
    json ToJSON() override;

    /**
     * @copydoc Device::FromJSON()
     * Equivalent to @ref Set() followed by @ref SaveInit()
     */
    void FromJSON(const json& j) override;

    void DownloadServiceReport(std::string out_file);

#ifdef BUILD_MODULE_CRYPTO
    /**
     * @brief Provides access to the SealedBox. The SealedBox is used to for
     * accessing the password protected features of the device
     */
    std::shared_ptr<O3RSealedBox> SealedBox();
#endif

  private:
    class Impl;
    std::shared_ptr<Impl> pImpl;

#ifdef BUILD_MODULE_CRYPTO
    friend class O3RSealedBox;
#endif
  }; // end: class O3R

#ifdef BUILD_MODULE_CRYPTO
  /**
   * @ingroup Device
   *
   * @brief Provides access to the O3R's SealedBox.
   * The SealedBox is used to for accessing the password protected features of
   * the device
   */
  class IFM3D_EXPORT O3RSealedBox
  {
  public:
    using Ptr = std::shared_ptr<O3RSealedBox>;

    O3RSealedBox(std::shared_ptr<O3R::Impl> p_impl);
    ~O3RSealedBox();

    /**
     * @brief Sets the password for the device
     *
     * @param new_password The new password for the device
     * @param old_password The old password for the device. Required if the
     * device is currently locked
     */
    void SetPassword(const std::string& new_password,
                     std::optional<std::string> old_password = std::nullopt);

    /**
     * @brief Returns whether the device is password protected
     *
     * @return true if the device is password protected
     */
    bool IsPasswordProtected();

    /**
     * @brief Removes the password protection from the device
     *
     * @param password The current password for the device
     */
    void RemovePassword(std::string password);

    /**
     * @brief Sets the configuration of the device. This allows setting
     * password protected parameters.
     *
     * @param password The current password for the device
     * @param configuration The new configuration for the device
     */
    void Set(const std::string& password, const json& configuration);

    /**
     * @brief Get the public key of the device used for sending encrypted
     * messages
     *
     * @return The encrypted message
     */
    std::vector<uint8_t> GetPublicKey();

  private:
    std::shared_ptr<O3R::Impl> pImpl;
  };
#endif
}

#endif // IFM3D_DEVICE_O3R_H