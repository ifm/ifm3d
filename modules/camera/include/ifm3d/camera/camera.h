// -*- c++ -*-
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

#ifndef __IFM3D_CAMERA_CAMERA_H__
#define __IFM3D_CAMERA_CAMERA_H__

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <ifm3d/contrib/nlohmann/json.hpp>
#include <ifm3d/camera/camera_export.h>

using json = nlohmann::json;

namespace ifm3d
{
  extern IFM3D_CAMERA_EXPORT const std::string DEFAULT_IP;
  extern IFM3D_CAMERA_EXPORT const std::uint16_t DEFAULT_XMLRPC_PORT;
  extern IFM3D_CAMERA_EXPORT const int DEFAULT_PCIC_PORT;
  extern IFM3D_CAMERA_EXPORT const std::string DEFAULT_PASSWORD;
  extern IFM3D_CAMERA_EXPORT const int MAX_HEARTBEAT;
  extern IFM3D_CAMERA_EXPORT const std::size_t SESSION_ID_SZ;
  extern IFM3D_CAMERA_EXPORT const std::string DEFAULT_SESSION_ID;
  extern IFM3D_CAMERA_EXPORT const std::string DEFAULT_APPLICATION_TYPE;

  extern IFM3D_CAMERA_EXPORT const int DEV_O3D_MIN;
  extern IFM3D_CAMERA_EXPORT const int DEV_O3D_MAX;
  extern IFM3D_CAMERA_EXPORT const int DEV_O3X_MIN;
  extern IFM3D_CAMERA_EXPORT const int DEV_O3X_MAX;
  extern IFM3D_CAMERA_EXPORT const std::string ASSUME_DEVICE;

  extern IFM3D_CAMERA_EXPORT const unsigned int O3D_TIME_SUPPORT_MAJOR;
  extern IFM3D_CAMERA_EXPORT const unsigned int O3D_TIME_SUPPORT_MINOR;
  extern IFM3D_CAMERA_EXPORT const unsigned int O3D_TIME_SUPPORT_PATCH;

  extern IFM3D_CAMERA_EXPORT const unsigned int O3D_TMP_PARAMS_SUPPORT_MAJOR;
  extern IFM3D_CAMERA_EXPORT const unsigned int O3D_TMP_PARAMS_SUPPORT_MINOR;
  extern IFM3D_CAMERA_EXPORT const unsigned int O3D_TMP_PARAMS_SUPPORT_PATCH;

  extern IFM3D_CAMERA_EXPORT const unsigned int O3D_INTRINSIC_PARAM_SUPPORT_MAJOR;
  extern IFM3D_CAMERA_EXPORT const unsigned int O3D_INTRINSIC_PARAM_SUPPORT_MINOR;
  extern IFM3D_CAMERA_EXPORT const unsigned int O3D_INTRINSIC_PARAM_SUPPORT_PATCH;

  extern IFM3D_CAMERA_EXPORT const unsigned int O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR;
  extern IFM3D_CAMERA_EXPORT const unsigned int O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR;
  extern IFM3D_CAMERA_EXPORT const unsigned int O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH;

  /**
   * Software interface to an ifm 3D camera
   *
   * The `Camera` class implements the underlying network protocol for
   * communicating with the ifm hardware. Via this communication layer, this
   * class exposes objects that can be used to mutate and tune the camera
   * parameters including those of the underlying pmd imager.
   */
  class Camera
  {
  public:
    using Ptr = std::shared_ptr<Camera>;

    /**
     * Camera boot up modes:
     *
     * Productive: the normal runtime firmware comes up
     * Recovery: allows you to flash new firmware
     */
    enum class boot_mode : int { PRODUCTIVE = 0, RECOVERY = 1 };

    /**
     * Camera operating modes: run (streaming pixel data), edit (configuring
     * the device/applications).
     */
    enum class operating_mode : int { RUN = 0, EDIT = 1 };

    /**
     * Image acquisition trigger modes
     */
    enum class trigger_mode : int { FREE_RUN = 1, SW = 2 };

    /**
     * Import flags used when importing a Vision Assistant configuration
     */
    enum class import_flags : int { GLOBAL = 0x1, NET = 0x2, APPS = 0x10 };

    /**
     * Convenience constants for spatial filter types
     */
    enum class spatial_filter : int
    { OFF = 0x0, MEDIAN = 0x1, MEAN = 0x2, BILATERAL = 0x3 };

    /**
     * Convenience constants for temporal filter types
     */
    enum class temporal_filter : int
    { OFF = 0x0, MEAN = 0x1, ADAPTIVE_EXP = 0x2 };

    /**
     * Convenient constants for median filter mask sizes
     */
    enum class mfilt_mask_size : int { _3x3 = 0, _5x5 = 1};

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
     * @param[in] ip The ip address of the camera
     * @param[in] xmlrpc_port The tcp port the sensor's XMLRPC server is
     *                        listening on
     * @param[in] password Password required for establishing an "edit session"
     *                     with the sensor. Edit sessions allow for mutating
     *                     camera parameters and persisting those changes.
     */
    static Ptr
    MakeShared(const std::string& ip = ifm3d::DEFAULT_IP,
               const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT,
               const std::string& password = ifm3d::DEFAULT_PASSWORD);

    /**
     * Initializes the camera interface utilizing library defaults
     * for password, ip, and xmlrpc port unless explicitly passed in.
     *
     * @param[in] ip The ip address of the camera
     * @param[in] xmlrpc_port The tcp port the sensor's XMLRPC server is
     *                        listening on
     * @param[in] password Password required for establishing an "edit session"
     *                     with the sensor. Edit sessions allow for mutating
     *                     camera parameters and persisting those changes.
     */
    Camera(const std::string& ip = ifm3d::DEFAULT_IP,
           const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT,
           const std::string& password = ifm3d::DEFAULT_PASSWORD);

    /**
     * The dtor will cancel any open edit sessions with the camera.
     */
    virtual ~Camera();

    // Based on our mileage with `libo3d3xx`, disabling copy and move semantics
    // on the camera class has not been an issue, so, we do that here too.
    Camera(Camera&&) = delete;
    Camera& operator=(Camera&&) = delete;
    Camera(Camera&) = delete;
    Camera& operator=(Camera&) = delete;

    // Accessors/Mutators

    /** The IP address associated with this Camera instance */
    virtual std::string IP();

    /** The XMLRPC Port associated with this Camera instance */
    virtual std::uint16_t XMLRPCPort();

    /** The password associated with this Camera instance */
    virtual std::string Password();

    /** Retrieves the active session id */
    virtual std::string SessionID();

    /**
     * Requests an edit-mode session with the camera.
     *
     * In order to (permanently) mutate parameters on the camera, an edit
     * session needs to be established. Only a single edit sesson may be
     * established at any one time with the camera (think of it as a global
     * mutex on the camera state -- except if you ask for the mutex and it is
     * already taken, an exception will be thrown).
     *
     * Most typical use-cases for end-users will not involve establishing an
     * edit-session with the camera. To mutate camera parameters, the
     * `FromJSON` family of functions should be used, which, under-the-hood, on
     * the user's behalf, will establish the edit session and gracefully close
     * it. There is an exception. For users who plan to modulate imager
     * parameters (temporary parameters) on the fly while running the
     * framegrabber, managing the session manually is necessary. For this
     * reason, we expose this method in the public `Camera` interface.
     *
     * NOTE: The session timeout is implicitly set to `ifm3d::MAX_HEARTBEAT`
     * after the session has been successfully established.
     *
     * @return The session id issued or accepted by the camera (see
     *         IFM3D_SESSION_ID environment variable)
     *
     * @throws ifm3d::error_t if an error is encountered.
     */
    virtual std::string RequestSession();

    /**
     * Explictly stops the current session with the sensor.
     *
     * NOTE: This function returns a boolean indicating the success/failure of
     * cancelling the session. The reason we return a bool and explicitly
     * supress exceptions is because we want to cancel any open sessions in the
     * camera dtor and we do not want to throw in the dtor.
     *
     * @return true if the session was cancelled properly, false if an
     * exception was caught trying to close the session. Details will be
     * logged.
     */
    virtual bool CancelSession();

    /**
     * Attempts to cancel a session with a particular session id.
     *
     * @return true if the session was cancelled properly, false if an
     * exception was caught trying to close the session. Details will be
     * logged.
     */
    virtual bool CancelSession(const std::string& sid);

    /**
     * Heartbeat messages are used to keep a session with the sensor
     * alive. This function sends a heartbeat message to the sensor and sets
     * when the next heartbeat message is required.
     *
     * @param[in] hb The time (seconds) of when the next heartbeat message will
     *               be required.
     *
     * @return The current timeout interval in seconds for heartbeat messages.
     *
     * @throw ifm3d::error_t upon error
     */
    virtual int Heartbeat(int hb);

    /**
     * Sets temporary application parameters in run mode.
     *
     * The changes are not persistent and are lost when entering edit mode or
     * turning the device off. The parameters "ExposureTime" and
     * "ExposureTimeRatio" of the imager configuration are supported. All
     * additional parameters are ignored (for now). Exposure times are clamped
     * to their allowed range, depending on the exposure mode. The user must
     * provide the complete set of parameters depending on the exposure mode,
     * i.e., "ExposureTime" only for single exposure modes and both
     * "ExposureTime" and "ExposureTimeRatio" for double exposure
     * modes. Otherwise, behavior is undefined.
     *
     * @param[in] params The parameters to set on the camera.
     *
     * @throw ifm3d::error_t upon error
     */
    virtual void SetTemporaryApplicationParameters(
      const std::unordered_map<std::string, std::string>& params);

    /**
     * Sends a S/W trigger to the camera over XMLRPC.
     *
     * The O3X does not S/W trigger over PCIC, so, this function
     * has been developed specficially for it. For the O3D, this is a NOOP.
     */
    virtual void ForceTrigger();

    /**
     * Reboot the sensor
     *
     * @param[in] mode The system mode to boot into upon restart of the sensor
     * @throw ifm3d::error_t upon error
     */
    virtual void
    Reboot(const boot_mode& mode = ifm3d::Camera::boot_mode::PRODUCTIVE);

    /**
     * Returns the integer index of the active application. A negative number
     * indicates no application is marked as active on the sensor.
     */
    virtual int ActiveApplication();

    /**
     * This is a convenience function for extracting out the device type of the
     * connected camera. The primary intention of this function is for internal
     * usage (i.e., to trigger conditional logic based on the model hardware
     * we are talking to) however, it will likely be useful in
     * application-level logic as well, so, it is available in the public
     * interface.
     *
     * @param[in] use_cached If set to true, a cached lookup of the device
     *                       type will be used as the return value. If false,
     *                       it will make a network call to the camera to get
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
     * Checks if the device is in the O3X family
     *
     * @return true for an O3X device
     */
    virtual bool IsO3X();

    /**
     * Checks if the device is in the O3D family
     *
     * @return true for an O3D device
     */
    virtual bool IsO3D();

    /**
     * Convenience accessor for extracting a device parameters
     * (i.e., no edit session created on the camera)
     */
    virtual std::string DeviceParameter(const std::string& key);

    /**
     * Delivers the trace log from the camera
     * A session is not required to call this function.
     *
     * @return A `vector' of `std::string' for each entry in the tracelog
     *
     * @throw ifm3d::error_t upon error
     */
     virtual std::vector<std::string> TraceLogs(int count);

    /**
     * Delivers basic information about all applications stored on the device.
     * A call to this function does not require establishing a session with the
     * camera.
     *
     * The returned information is encoded as an array of JSON objects.
     * Each object in the array is basically a dictionary with the following
     * keys: 'index', 'id', 'name', 'description', 'active'
     *
     * @return A JSON encoding of the application information
     * @throw ifm3d::error_t upon error
     */
    virtual json ApplicationList();

    /**
     * Lists the valid application types supported by the sensor.
     *
     * @return A vector of strings listing the available types of applications
     *        supported by the sensor. Each element of the vector is a string
     *        suitable to passing to `CreateApplication`.
     *
     * @throw ifm3d::error_t upon error
     */
    virtual std::vector<std::string> ApplicationTypes();

    /**
     * Lists the valid imager types supported by the sensor.
     *
     * @return A vector of strings listing the available types of imagers
     *         supported by the sensor.
     *
     * @throw ifm3d::error_t upon error
     */
    virtual std::vector<std::string> ImagerTypes();

    /**
     * Creates a new application by copying the configuration of another
     * application. The device will generate an ID for the new application and
     * put it on a free index.
     *
     * @param[in] idx The index of the application to copy
     *
     * @return Index of the new application
     *
     * @throw ifm3d::error_t upon error
     */
    virtual int CopyApplication(int idx);

    /**
     * Creates a new application on the camera of the given type.
     *
     * To figure out valid `type`s, you should call the
     *`AvailableApplicationTypes()` method.
     *
     * Upon creation of the application, the embedded device will initialize
     * all parameters as necessary based on the type. However, based on the
     * type, the application may not be in an _activatable_ state. That is, it
     * can be created and saved on the device, but it cannot be marked as
     * active.
     *
     * @param[in] type The (optional) application type to create. By default,
     *                 it will create a new "Camera" application.
     *
     * @return The index of the new application.
     */
    virtual int CreateApplication(
            const std::string& type = DEFAULT_APPLICATION_TYPE);

    /**
     * Deletes the application at the specified index from the sensor.
     *
     * @param[in] idx The index of the application to delete
     * throw ifm3d::error_t upon error
     */
    virtual void DeleteApplication(int idx);

    /**
     * Explicitly sets the current time on the camera.
     *
     * @param[in] epoch_secs Time since the Unix epoch in seconds.
     *            A value less than 0 will implicitly set the time
     *            to the current system time.
     */
    virtual void SetCurrentTime(int epoch_secs = -1);

    /**
     * Sets the camera configuration back to the state in which it shipped from
     * the ifm factory.
     */
    virtual void FactoryReset();

    /**
     * For cameras that support fetching the Unit Vectors over XML-RPC,
     * this function will return those data as a binary blob.
     */
    virtual std::vector<std::uint8_t> UnitVectors();

    /**
     * Exports the entire camera configuration in a format compatible with
     * Vision Assistant.
     */
    virtual std::vector<std::uint8_t> ExportIFMConfig();

    /**
     * Export the application at the specified index into a byte array suitable
     * for writing to a file. The exported bytes represent the IFM
     * serialization of an application.
     *
     * This function provides compatibility with tools like IFM's Vision
     * Assistant.
     *
     * @param[in] idx The index of the application to export.
     *
     * @return A vector of bytes representing the IFM serialization of the
     *         exported application.
     *
     * @throw ifm3d::error_t upon error
     */
    virtual std::vector<std::uint8_t> ExportIFMApp(int idx);

    /**
     * Imports an entire camera configuration from a format compatible with
     * Vision Assistant.
     */
    virtual void ImportIFMConfig(const std::vector<std::uint8_t>& bytes,
                                 std::uint16_t flags = 0x0);

    /**
     * Import the IFM-encoded application.
     *
     * This function provides compatibility with tools like IFM's Vision
     * Assistant. An application configuration exported from VA, can be
     * imported using this function.
     *
     * @param[in] bytes The raw bytes from the zip'd JSON file. NOTE: This
     *                  function will base64 encode the data for tranmission
     *                  over XML-RPC.
     *
     * @return The index of the imported application.
     */
    virtual int ImportIFMApp(const std::vector<std::uint8_t>& bytes);

    /**
     * Serializes the state of the camera to JSON.
     *
     * The JSON interface returned here is the excellent
     * <a href="https://github.com/nlohmann/json">JSON for Modern C++</a>.
     *
     * This function (along with its `std::string` equivalent `ToJSONStr()`)
     * provides the primary gateway into obtaining the current parameter
     * settings for the camera and PMD imager. Data returned from this function
     * can be manipulated as a `json` object, then fed into `FromJSON(...)` to
     * mutate parameter settings on the camera.
     *
     * @return A JSON object representation of the current state of the
     *         hardware.
     *
     * @throw ifm3d::error_t upon error
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
     * Configures the camera based on the parameter values of the passed in
     * JSON. This function is _the_ way to tune the
     * camera/application/imager/etc. parameters.
     *
     * @param[in] json A json object encoding a camera configuration to apply
     *                 to the hardware.
     *
     * @todo This needs to be fully documented!
     * Processing proceeds as follows:
     *
     * - Device parameters are processed and saved persistently
     *
     * @throw ifm3d::error_t upon error - if this throws an exception, you are
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
     * Sets or disable the password on the camera.
     *
     * @param[in] password is the password string. If the password is blank,
     *                     password is disabled
     *
     * @throw ifm3d::error_t upon error
     */
    virtual void SetPassword(std::string password = "");

    /**
     * Checks for a minimum ifm camera software version
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

  protected:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    /**
     * The cached device type of the connected device
     */
    std::string device_type_;

    /**
     * Handles parsing a selected sub-tree of a potential input JSON file,
     * setting the parameters as appropriate on the camera, and saving them
     * persistently.
     *
     * @param[in] j_curr The current configuration
     * @param[in] j_new  The desired configuration
     * @param[in] SetFunc The setter function to call for each parameter
     * @param[in] SaveFunc The function to call to persist the values
     * @param[in] name A descriptive name for the sub-tree (used to make
     *                 log messages useful).
     * @param[in] idx An application index to put into edit mode prior to
     *                setting parameters.
     */
    void FromJSON_(const json& j_curr,
                   const json& j_new,
                   std::function<void(const std::string&,
                                      const std::string&)> SetFunc,
                   std::function<void()> SaveFunc,
                   const std::string& name,
                   int idx = -1);

    /**
     *  Implements the serialization of the camera state to JSON.
     *  @param[in] open_session if false function will work
                   on already opened session
     *  @return A JSON object representation of the current camera state.
     */
    json ToJSON_(const bool open_session = true);

  }; // end: class Camera

  /**
   * Camera specialization for O3D
   */
  class O3DCamera : public Camera
  {
  public:
    using Ptr = std::shared_ptr<O3DCamera>;
    O3DCamera(const std::string& ip = ifm3d::DEFAULT_IP,
              const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT,
              const std::string& password = ifm3d::DEFAULT_PASSWORD);

    virtual ~O3DCamera();
    O3DCamera(O3DCamera&&) = delete;
    O3DCamera& operator=(O3DCamera&&) = delete;
    O3DCamera(O3DCamera&) = delete;
    O3DCamera& operator=(O3DCamera&) = delete;

    virtual bool IsO3X();
    virtual bool IsO3D();
  }; // end: class O3DCamera

  /**
   * Camera specialization for O3X
   */
  class O3XCamera : public Camera
  {
  public:
    using Ptr = std::shared_ptr<O3XCamera>;
    O3XCamera(const std::string& ip = ifm3d::DEFAULT_IP,
              const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT,
              const std::string& password = ifm3d::DEFAULT_PASSWORD);

    virtual ~O3XCamera();
    O3XCamera(O3XCamera&&) = delete;
    O3XCamera& operator=(O3XCamera&&) = delete;
    O3XCamera(O3XCamera&) = delete;
    O3XCamera& operator=(O3XCamera&) = delete;

    virtual bool IsO3X();
    virtual bool IsO3D();
  }; // end: class O3XCamera

} // end: namespace ifm3d

#endif // __IFM3D_CAMERA_CAMERA_H__
