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
#include <ifm3d/contrib/json.hpp>

using json = nlohmann::json;

namespace ifm3d
{
  extern const std::string DEFAULT_IP;
  extern const std::uint16_t DEFAULT_XMLRPC_PORT;
  extern const std::string DEFAULT_PASSWORD;
  extern const int MAX_HEARTBEAT;

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
     * Import flags used when importing a Vision Assistant configuration
     */
    enum class import_flags : int { GLOBAL = 0x1, NET = 0x2, APPS = 0x10 };

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
    std::string IP();

    /** The XMLRPC Port associated with this Camera instance */
    std::uint16_t XMLRPCPort();

    /** The password associated with this Camera instance */
    std::string Password();

    /** Retrieves the active session id */
    std::string SessionID();

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
     * @return The session id issued by the camera.
     *
     * @throws ifm3d::error_t if an error is encountered.
     */
    std::string RequestSession();

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
    bool CancelSession();

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
    int Heartbeat(int hb);

    /**
     * Reboot the sensor
     *
     * @param[in] mode The system mode to boot into upon restart of the sensor
     * @throw ifm3d::error_t upon error
     */
    void Reboot(const boot_mode& mode = ifm3d::Camera::boot_mode::PRODUCTIVE);

    /**
     * Returns the integer index of the active application. A negative number
     * indicates no application is marked as active on the sensor.
     */
    int ActiveApplication();

    /**
     * This is a convenience function for extracting out the article number of
     * the connected camera. The primary intention of the function is for
     * internal usage (i.e., triggering some conditional logic based on the
     * model hardware we are talking to) however, it will likely be useful in
     * other scenarios as well, so, it is available in the public interface.
     */
    std::string ArticleNumber();

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
    json ApplicationList();

    /**
     * Lists the valid application types supported by the sensor.
     *
     * @return A vector of strings listing the available types of applications
     *        supported by the sensor. Each element of the vector is a string
     *        suitable to passing to `CreateApplication`.
     *
     * @throw ifm3d::error_t upon error
     */
    std::vector<std::string> ApplicationTypes();

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
    int CopyApplication(int idx);

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
    int CreateApplication(const std::string& type = "Camera");

    /**
     * Deletes the application at the specified index from the sensor.
     *
     * @param[in] idx The index of the application to delete
     * throw ifm3d::error_t upon error
     */
    void DeleteApplication(int idx);

    /**
     * Sets the camera configuration back to the state in which it shipped from
     * the ifm factory.
     */
    void FactoryReset();

    /**
     * Exports the entire camera configuration in a format compatible with
     * Vision Assistant.
     */
    std::vector<std::uint8_t> ExportIFMConfig();

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
    std::vector<std::uint8_t> ExportIFMApp(int idx);

    /**
     * Imports an entire camera configuration from a format compatible with
     * Vision Assistant.
     */
    void ImportIFMConfig(const std::vector<std::uint8_t>& bytes,
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
    int ImportIFMApp(const std::vector<std::uint8_t>& bytes);

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
    json ToJSON();

    /**
     * A stringified version of the JSON object returned by `ToJSON()`.
     *
     * @return A string version of the Camera's JSON representation.
     *
     * @see ToJSON
     */
    std::string ToJSONStr();

    /**
     * Configures the camera based on the parameter values of the passed in
     * JSON. This function is _the_ way to tune the
     * camera/application/imager/etc. parameters.
     *
     * @param[in] json A json object encoding a camera configuration to apply
     *                 to the hardware.
     *
     * Processing proceeds as follows:
     *
     * - Device parameters are processed and saved persistently
     *
     * @throw ifm3d::error_t upon error - if this throws an exception, you are
     *        encouraged to check the log file as a best effort is made to be
     *        as descriptive as possible as to the specific error that has
     *        occured.
     */
    void FromJSON(const json& j);

    /**
     * Accepts a string with properly formatted/escaped JSON text, converts it
     * to a `json` object, and call `FromJSON()` on it.
     *
     * @see FromJSON
     */
    void FromJSONStr(const std::string& jstr);

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

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
     */
    void FromJSON_(const json& j_curr,
                   const json& j_new,
                   std::function<void(const std::string&,
                                      const std::string&)> SetFunc,
                   std::function<void()> SaveFunc,
                   const std::string& name);

  }; // end: class Camera

} // end: namespace ifm3d

#endif // __IFM3D_CAMERA_CAMERA_H__
