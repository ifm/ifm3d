/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_CAMERA_CAMERA_H
#define IFM3D_CAMERA_CAMERA_H

#include <ifm3d/camera/camera_base.h>

namespace ifm3d
{
  class Camera : public CameraBase
  {
  public:
    using Ptr = std::shared_ptr<Camera>;

    using boot_mode = ifm3d::CameraBase::boot_mode;
    using operating_mode = ifm3d::CameraBase::operating_mode;
    using trigger_mode = ifm3d::CameraBase::trigger_mode;
    using import_flags = ifm3d::CameraBase::import_flags;
    using spatial_filter = ifm3d::CameraBase::spatial_filter;
    using temporal_filter = ifm3d::CameraBase::temporal_filter;
    using mfilt_mask_size = ifm3d::CameraBase::mfilt_mask_size;
    using device_family = ifm3d::CameraBase::device_family;

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
    static Ptr MakeShared(
      const std::string& ip = ifm3d::DEFAULT_IP,
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

    virtual ~Camera();

    /** The password associated with this Camera instance */
    virtual std::string Password();

    /** Retrieves the active session id */
    virtual std::string SessionID();

    /**
     * Sets the camera configuration back to the state in which it shipped from
     * the ifm factory.
     */
    virtual void FactoryReset();

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

    virtual std::unordered_map<std::string, std::string> NetInfo();
    virtual std::unordered_map<std::string, std::string> TimeInfo();

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
     * Returns the integer index of the active application. A negative number
     * indicates no application is marked as active on the sensor.
     */
    virtual int ActiveApplication();

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
     * Sets or disable the password on the camera.
     *
     * @param[in] password is the password string. If the password is blank,
     *                     password is disabled
     *
     * @throw ifm3d::error_t upon error
     */
    virtual void SetPassword(std::string password = "");

    json ToJSON() override;
    void FromJSON(const json& j) override;
    void ForceTrigger() override;

  protected:
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
     * @param[in] idx An application index to put into edit mode prior to
     *                setting parameters.
     */
    void FromJSON_(
      const json& j_curr,
      const json& j_new,
      std::function<void(const std::string&, const std::string&)> SetFunc,
      std::function<void()> SaveFunc,
      const std::string& name,
      int idx = -1);

    json ToJSON_(const bool open_session = true);

<<<<<<< HEAD
    json getApplicationInfosToJSON();
  };
}
=======
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

    std::unordered_map<std::string, std::string> TimeInfo() override;
    bool IsO3X() override;
    bool IsO3D() override;
    bool IsO3R() override;
	ifm3d::Camera::sw_version SwUpdateVersion() override;
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

    bool IsO3X() override;
    bool IsO3D() override;
    bool IsO3R() override;

    ifm3d::Camera::sw_version SwUpdateVersion() override;
  }; // end: class O3XCamera

  /**
   * Camera specialization for O3R
   */
  class O3RCamera : public Camera
  {
  public:
    using Ptr = std::shared_ptr<O3RCamera>;
    O3RCamera(const std::string& ip = ifm3d::DEFAULT_IP,
              const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT,
              const std::string& password = ifm3d::DEFAULT_PASSWORD);

    virtual ~O3RCamera();
    O3RCamera(O3RCamera&&) = delete;
    O3RCamera& operator=(O3RCamera&&) = delete;
    O3RCamera(O3RCamera&) = delete;
    O3RCamera& operator=(O3RCamera&) = delete;

    bool IsO3X() override;
    bool IsO3D() override;
    bool IsO3R() override;

    json ToJSON() override;
    void FromJSON(const json& j) override;
    ifm3d::Camera::sw_version SwUpdateVersion() override;
  }; // end: class O3RCamera

} // end: namespace ifm3d
>>>>>>> c705f0ae4... Added Supported swVersion to camera class

#endif // IFM3D_CAMERA_CAMERA_H