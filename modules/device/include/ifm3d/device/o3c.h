/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_O3C_H
#define IFM3D_DEVICE_O3C_H

#include <ifm3d/common/features.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3r.h>
#include <sstream>
// #ifdef BUILD_MODULE_CRYPTO
// #  include <ifm3d/crypto/crypto.h>
// #endif
#include <ifm3d/device/pcic_command.h>
#include <initializer_list>
#include <variant>

namespace ifm3d
{
  static const int NET_WAIT_O3C_SET =
    std::getenv("IFM3D_NET_WAIT_O3C_SET") == nullptr ?
      15000 :
      std::stoi(std::getenv("IFM3D_NET_WAIT_O3C_SET"));

  /** @ingroup Device
   *
   * Device specialization for O3C
   */
  class IFM3D_EXPORT O3C : public O3R
  {
  public:
    using Ptr = std::shared_ptr<O3C>;
    O3C(const std::string& ip = ifm3d::DEFAULT_IP,
        std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT);

    ~O3C() override;
    O3C(O3C&&) = delete;
    O3C& operator=(O3C&&) = delete;
    O3C(O3C&) = delete;
    O3C& operator=(O3C&) = delete;

    /**
     * Sets the device configuration back to the state in which it shipped from
     * the ifm factory.
     *
     * @param[in] keepNetworkSettings a bool indicating wether to keep the
     * current network settings
     */
    void FactoryReset(bool keep_network_settings) override;
    void Reboot(
      const BootMode& mode = ifm3d::Device::BootMode::PRODUCTIVE) override;

    DeviceFamily WhoAmI() override;
    ifm3d::Device::SWUVersion SwUpdateVersion() override;

    /**
     * @copydoc Device::ToJSON()
     * Equivalent to the @ref Get() method
     */
    ifm3d::json ToJSON() override;

    /**
     * @copydoc Device::FromJSON()
     * Equivalent to @ref Set() followed by @ref SaveInit()
     */
    void FromJSON(const json& j) override;

    // O3C-specific PCIC command for temporary application parameters
    struct SetTemporaryApplicationParameter : public PCICCommand
    {
      enum Parameter : uint16_t
      {
        ODS_OVERHANGING_LOAD = 2003,
        ODS_ZONE_SET = 2101,
        ODS_MAXIMUM_HEIGHT = 2102,
        ODS_MOTION_DATA = 2103,
        PDS_GET_PALLET = 2200,
        PDS_GET_ITEM = 2201,
        PDS_GET_RACK = 2202,
        PDS_VOL_CHECK = 2203,
      };

      Parameter parameter;
      std::vector<std::uint8_t> data;

      SetTemporaryApplicationParameter(
        Parameter param,
        std::vector<std::uint8_t> parameter_data)
        : parameter(param),
          data(std::move(parameter_data))
      {}

      std::vector<std::uint8_t>
      SerializeData() const override
      {
        std::vector<std::uint8_t> payload;

        // 'f' command prefix
        payload.push_back(static_cast<uint8_t>('f'));

        // parameter ID as 5-digit ASCII string padded with zeros
        std::ostringstream param;
        param << std::setw(5) << std::setfill('0')
              << static_cast<uint16_t>(parameter);
        const std::string paramStr = param.str(); // e.g., "02103"
        payload.insert(payload.end(), paramStr.begin(), paramStr.end());

        // reserved string "#00000" as bytes
        const std::string reserved = "#00000";
        payload.insert(payload.end(), reserved.begin(), reserved.end());

        // append user data bytes
        payload.insert(payload.end(), data.begin(), data.end());

        return payload;
      }
    };
  };
}

#endif // IFM3D_DEVICE_O3C_H