// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/// @file
#ifndef IFM3D_DEVICE_DEVICE_H
#define IFM3D_DEVICE_DEVICE_H

#include <cstdint>
#include <string>
#include <vector>
#include <ifm3d/common/json.hpp>
#include <ifm3d/device/device_export.h>
#include <ifm3d/device/ifm_network_device.h>
#include <ifm3d/device/semver.h>

/// @brief
namespace ifm3d
{
  extern IFM3D_DEVICE_EXPORT const std::string DEFAULT_IP;
  extern IFM3D_DEVICE_EXPORT const std::uint16_t DEFAULT_XMLRPC_PORT;
  extern IFM3D_DEVICE_EXPORT const int DEFAULT_PCIC_PORT;
  extern IFM3D_DEVICE_EXPORT const std::uint16_t PCIC_PORT;
  extern IFM3D_DEVICE_EXPORT const std::string DEFAULT_PASSWORD;
  extern IFM3D_DEVICE_EXPORT const int MAX_HEARTBEAT;
  extern IFM3D_DEVICE_EXPORT const std::size_t SESSION_ID_SZ;
  extern IFM3D_DEVICE_EXPORT const std::string DEFAULT_SESSION_ID;
  extern IFM3D_DEVICE_EXPORT const std::string DEFAULT_APPLICATION_TYPE;
  extern IFM3D_DEVICE_EXPORT const long DEFAULT_CURL_CONNECT_TIMEOUT;
  extern IFM3D_DEVICE_EXPORT const long DEFAULT_CURL_TRANSACTION_TIMEOUT;

  extern IFM3D_DEVICE_EXPORT const int DEV_O3D_MIN;
  extern IFM3D_DEVICE_EXPORT const int DEV_O3D_MAX;
  extern IFM3D_DEVICE_EXPORT const int DEV_O3R_MIN;
  extern IFM3D_DEVICE_EXPORT const int DEV_O3R_MAX;
  extern IFM3D_DEVICE_EXPORT const int DEV_O3X_MIN;
  extern IFM3D_DEVICE_EXPORT const int DEV_O3X_MAX;
  extern IFM3D_DEVICE_EXPORT const std::string ASSUME_DEVICE;

  extern IFM3D_DEVICE_EXPORT const unsigned int O3D_TIME_SUPPORT_MAJOR;
  extern IFM3D_DEVICE_EXPORT const unsigned int O3D_TIME_SUPPORT_MINOR;
  extern IFM3D_DEVICE_EXPORT const unsigned int O3D_TIME_SUPPORT_PATCH;

  extern IFM3D_DEVICE_EXPORT const unsigned int O3D_TMP_PARAMS_SUPPORT_MAJOR;
  extern IFM3D_DEVICE_EXPORT const unsigned int O3D_TMP_PARAMS_SUPPORT_MINOR;
  extern IFM3D_DEVICE_EXPORT const unsigned int O3D_TMP_PARAMS_SUPPORT_PATCH;

  extern IFM3D_DEVICE_EXPORT const unsigned int
    O3D_INTRINSIC_PARAM_SUPPORT_MAJOR;
  extern IFM3D_DEVICE_EXPORT const unsigned int
    O3D_INTRINSIC_PARAM_SUPPORT_MINOR;
  extern IFM3D_DEVICE_EXPORT const unsigned int
    O3D_INTRINSIC_PARAM_SUPPORT_PATCH;

  extern IFM3D_DEVICE_EXPORT const unsigned int
    O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR;
  extern IFM3D_DEVICE_EXPORT const unsigned int
    O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR;
  extern IFM3D_DEVICE_EXPORT const unsigned int
    O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH;

  extern IFM3D_DEVICE_EXPORT const unsigned int
    O3X_DISTANCE_NOISE_IMAGE_SUPPORT_MAJOR;
  extern IFM3D_DEVICE_EXPORT const unsigned int
    O3X_DISTANCE_NOISE_IMAGE_SUPPORT_MINOR;
  extern IFM3D_DEVICE_EXPORT const unsigned int
    O3X_DISTANCE_NOISE_IMAGE_SUPPORT_PATCH;

  /// @brief Pixel format correspondence
  enum class pixel_format : std::uint32_t
  {
    FORMAT_8U = 0,
    FORMAT_8S = 1,
    FORMAT_16U = 2,
    FORMAT_16S = 3,
    FORMAT_32U = 4,
    FORMAT_32S = 5,
    FORMAT_32F = 6,
    FORMAT_64U = 7,
    FORMAT_64F = 8,
    FORMAT_16U2 = 9,
    FORMAT_32F3 = 10
  };

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  enum class image_chunk : std::uint32_t
  {
    RADIAL_DISTANCE_IMAGE = 100,
    NORM_AMPLITUDE_IMAGE = 101, // normalized amplitude
    AMPLITUDE_IMAGE = 103,
    GRAYSCALE_IMAGE = 104, // ambient light
    RADIAL_DISTANCE_NOISE = 105,
    REFLECTIVITY = 107,
    CARTESIAN_X_COMPONENT = 200,
    CARTESIAN_Y_COMPONENT = 201,
    CARTESIAN_Z_COMPONENT = 202,
    CARTESIAN_ALL = 203,
    UNIT_VECTOR_ALL = 223,
    MONOCHROM_2D_12BIT = 250,
    MONOCHROM_2D = 251,
    JPEG_IMAGE = 260,
    CONFIDENCE_IMAGE = 300,
    DIAGNOSTIC = 302,
    JSON_DIAGNOSTIC = 305,
    EXTRINSIC_CALIB = 400,
    INTRINSIC_CALIB = 401,
    INVERSE_INTRINSIC_CALIBRATION = 402,
    TOF_INFO = 420,
    O3R_DISTANCE_IMAGE_INFO [[deprecated]] =
      static_cast<uint32_t>(ifm3d::image_chunk::TOF_INFO),
    RGB_INFO = 421,
    O3R_RGB_IMAGE_INFO [[deprecated]] =
      static_cast<uint32_t>(ifm3d::image_chunk::RGB_INFO),
    JSON_MODEL = 500,
    ALGO_DEBUG = 900,
    O3R_ODS_OCCUPANCY_GRID = 1000,
    O3R_ODS_INFO = 1001,
    O3R_RESULT_JSON = 1002,
    O3R_RESULT_ARRAY2D = 1003,
    O3R_RESULT_IMU = 1004,
  };
#endif // DOXYGEN_SHOULD_SKIP_THIS

  class XMLRPCWrapper;

  /** @ingroup Device
   *
   * Software interface to an ifm 3D device
   *
   * The `Device` class implements the underlying network protocol for
   * communicating with the ifm hardware. Via this communication layer, this
   * class exposes objects that can be used to mutate and tune the device
   * parameters including those of the underlying pmd imager.
   */
  class IFM3D_DEVICE_EXPORT Device
  {
  public:
    using Ptr = std::shared_ptr<Device>;

    /**
     * Device boot up modes:
     *
     * Productive: the normal runtime firmware comes up
     * Recovery: allows you to flash new firmware
     */
    enum class boot_mode : int
    {
      PRODUCTIVE = 0,
      RECOVERY = 1
    };

    /**
     * Device operating modes: run (streaming pixel data), edit (configuring
     * the device/applications).
     */
    enum class operating_mode : int
    {
      RUN = 0,
      EDIT = 1
    };

    /**
     * Image acquisition trigger modes
     */
    enum class trigger_mode : int
    {
      FREE_RUN = 1,
      SW = 2
    };

    /**
     * Import flags used when importing a Vision Assistant configuration
     */
    enum class import_flags : int
    {
      GLOBAL = 0x1,
      NET = 0x2,
      APPS = 0x10
    };

    /**
     * Convenience constants for spatial filter types
     */
    enum class spatial_filter : int
    {
      OFF = 0x0,
      MEDIAN = 0x1,
      MEAN = 0x2,
      BILATERAL = 0x3
    };

    /**
     * Convenience constants for temporal filter types
     */
    enum class temporal_filter : int
    {
      OFF = 0x0,
      MEAN = 0x1,
      ADAPTIVE_EXP = 0x2
    };

    /**
     * Convenient constants for median filter mask sizes
     */
    enum class mfilt_mask_size : int
    {
      _3x3 = 0,
      _5x5 = 1
    };

    enum class device_family : int
    {
      UNKNOWN = 0,
      O3D = 1,
      O3X = 2,
      O3R = 3,
    };

    enum class swu_version : int
    {
      SWU_NOT_SUPPORTED = 0,
      SWU_V1 = 1,
      SWU_V2 = 2
    };

    /**
     * @brief This function Provides a convinent way to find all
     *   ifm devices on the network.
     * @return : vector of ip-address all the discovered devices
     *  on network.
     */
    static std::vector<ifm3d::IFMNetworkDevice> DeviceDiscovery();

    /**
     * Factory function for instantiating the proper subclass based on h/w
     * probing.
     *
     * This function provides a convenient way for users of the library to
     * write hardware independent code. This function probes the connected
     * hardware and returns a proper subclass based upon the returned
     * `DeviceType`. In the event that the hardware is not connected, the error
     * is trapped and an instance of the base class is returned. The net
     * result of not having an instance of a subclass is: 1) worse performance,
     * 2) errors will come back from the sensor rather than the library -- some
     * of which may be hard to debug.
     *
     * @param[in] ip The ip address of the device
     * @param[in] xmlrpc_port The tcp port the sensor's XMLRPC server is
     *                        listening on
     * @param[in] password Password required for establishing an "edit session"
     *                     with the sensor. Edit sessions allow for mutating
     *                     device parameters and persisting those changes.
     */
    static Ptr MakeShared(
      const std::string& ip = ifm3d::DEFAULT_IP,
      const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT,
      const std::string& password = ifm3d::DEFAULT_PASSWORD,
      bool throwIfUnavailable = true);

    /**
     * Initializes the device interface utilizing library defaults
     * for password, ip, and xmlrpc port unless explicitly passed in.
     *
     * @param[in] ip The ip address of the device
     * @param[in] xmlrpc_port The tcp port the sensor's XMLRPC server is
     *                        listening on
     * @param[in] password Password required for establishing an "edit session"
     *                     with the sensor. Edit sessions allow for mutating
     *                     device parameters and persisting those changes.
     */
    Device(const std::string& ip = ifm3d::DEFAULT_IP,
           const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT);

    /**
     * The dtor will cancel any open edit sessions with the device.
     */
    virtual ~Device();

    // Based on our mileage with `libo3d3xx`, disabling copy and move semantics
    // on the device class has not been an issue, so, we do that here too.
    Device(Device&&) = delete;
    Device& operator=(Device&&) = delete;
    Device(Device&) = delete;
    Device& operator=(Device&) = delete;

    // Accessors/Mutators

    /** The IP address associated with this Device instance */
    virtual std::string IP();

    /** The XMLRPC Port associated with this Device instance */
    virtual std::uint16_t XMLRPCPort();

    /**
     * Reboot the device
     *
     * @param[in] mode The system mode to boot into upon restart of the device
     * @throw ifm3d::Error upon error
     */
    virtual void Reboot(
      const boot_mode& mode = ifm3d::Device::boot_mode::PRODUCTIVE);

    /**
     * Sends a S/W trigger to the device over XMLRPC.
     *
     * The O3X does not S/W trigger over PCIC, so, this function
     * has been developed specficially for it. For other sensors, this is a
     * NOOP.
     */
    virtual void ForceTrigger();

    /**
     * This is a convenience function for extracting out the device type of the
     * connected device. The primary intention of this function is for internal
     * usage (i.e., to trigger conditional logic based on the model hardware
     * we are talking to) however, it will likely be useful in
     * application-level logic as well, so, it is available in the public
     * interface.
     *
     * @param[in] use_cached If set to true, a cached lookup of the device
     *                       type will be used as the return value. If false,
     *                       it will make a network call to the device to get
     *                       the "real" device type. The only reason for
     *                       setting this to `false` would be if you expect
     *                       over the lifetime of your camera instance that you
     *                       will swap out (for example) an O3D for an O3X (or
     *                       vice versa) -- literally, swapping out the network
     *                       cables while an object instance is still alive. If
     *                       that is not something you are worried about,
     *                       leaving this set to true should result in a
     *                       signficant performance increase.
     */
    virtual std::string DeviceType(bool use_cached = true);

    /**
     * This function can be used to retrieve the family of the connected device
     *
     * @return the device family of the connected device.
     */
    virtual device_family WhoAmI();

    /**
     * This is a convenience function for checking whether a device is one of
     * the specified device family
     *
     * @param[in] family The family to check for
     *
     * @return true if the device is part of the family
     */
    virtual bool AmI(device_family family);

    /**
     * Convenience accessor for extracting a device parameters
     * (i.e., no edit session created on the device)
     */
    virtual std::string DeviceParameter(const std::string& key);

    /**
     * Delivers the trace log from the device
     * A session is not required to call this function.
     *
     * @return A `vector' of `std::string' for each entry in the tracelog
     *
     * @throw ifm3d::Error upon error
     */
    virtual std::vector<std::string> TraceLogs(int count);

    /**
     * Serializes the state of the device to JSON.
     *
     * The JSON interface returned here is the excellent
     * <a href="https://github.com/nlohmann/json">JSON for Modern C++</a>.
     *
     * This function (along with its `std::string` equivalent `ToJSONStr()`)
     * provides the primary gateway into obtaining the current parameter
     * settings for the device and PMD imager. Data returned from this function
     * can be manipulated as a `json` object, then fed into `FromJSON(...)` to
     * mutate parameter settings on the device.
     *
     * @return A JSON object representation of the current state of the
     *         hardware.
     *
     * @throw ifm3d::Error upon error
     */
    virtual json ToJSON();

    /**
     * A stringified version of the JSON object returned by `ToJSON()`.
     *
     * @return A string version of the Camera's JSON representation.
     *
     * @see ToJSON
     */
    virtual std::string ToJSONStr();

    /**
     * Configures the device based on the parameter values of the passed in
     * JSON. This function is _the_ way to tune the
     * device/application/imager/etc. parameters.
     *
     * @param[in] json A json object encoding a camera configuration to apply
     *                 to the hardware.
     *
     * @todo This needs to be fully documented!
     * Processing proceeds as follows:
     *
     * - Device parameters are processed and saved persistently
     *
     * @throw ifm3d::Error upon error - if this throws an exception, you are
     *        encouraged to check the log file as a best effort is made to be
     *        as descriptive as possible as to the specific error that has
     *        occured.
     */
    virtual void FromJSON(const json& j);

    /**
     * Accepts a string with properly formatted/escaped JSON text, converts it
     * to a `json` object, and call `FromJSON()` on it.
     *
     * @see FromJSON
     */
    virtual void FromJSONStr(const std::string& jstr);

    /**
     * Checks for a minimum ifm device software version
     *  @param[in] major  Major version of software
     *  @param[in] minor  Minor Version of software
     *  @param[in] patch  Patch Number of software
     *
     * return True if current software version is greater
     * or equal to the  value passed
     */
    bool CheckMinimumFirmwareVersion(unsigned int major,
                                     unsigned int minor,
                                     unsigned int patch);

    /**
     * get the firmware version of the device
     */
    ifm3d::SemVer FirmwareVersion();

    /**
     * Checks the swupdater version supported by device
     *
     * @return sw_version supported by device
     */
    virtual ifm3d::Device::swu_version SwUpdateVersion();

  protected:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    /**
     * The cached device type of the connected device
     */
    std::string device_type_;

    /**
     *  Implements the serialization of the device state to JSON.
     *  @param[in] open_session if false function will work
                   on already opened session
     *  @return A JSON object representation of the current device state.
     */
    int DeviceID();
    bool checkDeviceID(int deviceID, int minID, int maxID);

    std::shared_ptr<XMLRPCWrapper> XWrapper();

  }; // end: class Device

} // end: namespace ifm3d

#endif // IFM3D_DEVICE_DEVICE_H
