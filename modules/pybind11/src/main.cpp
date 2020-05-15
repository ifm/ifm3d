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

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/opencv.h>

// Pybind must come after ifm3d because the pybind includes
// set some macros that conflict with the nlohmann json.hpp
// implementation on Windows builds
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "util.hpp"

namespace py = pybind11;

namespace ifm3d
{
  /**
   * Helper class for binding ifm3d::FrameGrabber
   *
   * Simple wrapper around ifm3d::FrameGrabber with the following additions:
   * - The WaitForFrame function signature is compatible with passing a bound
   *   ifm3d::OpenCVBuffer object from Python
   * - Addition of a Reset method which is used to emulate the .reset()
   *   semantics on the C++ shared_ptr when, for example, changing the imager
   *   mask.
   */
  class FrameGrabberWrapper
  {
  public:
    using Ptr = std::shared_ptr<FrameGrabberWrapper>;

    /**
     * Creates an internally-held ifm3d::FrameGrabber object with the
     * provided arguments (1:1 map to the ifm3d::FrameGrabber ctor)
     */
    FrameGrabberWrapper(ifm3d::Camera::Ptr cam,
                        std::uint16_t mask = ifm3d::DEFAULT_SCHEMA_MASK);

    // disable copy/move semantics
    FrameGrabberWrapper(FrameGrabberWrapper&&) = delete;
    FrameGrabberWrapper& operator=(FrameGrabberWrapper&&) = delete;
    FrameGrabberWrapper(FrameGrabberWrapper&) = delete;
    FrameGrabberWrapper& operator=(const FrameGrabberWrapper&) = delete;

    /**
     * Passthrough to ifm3d::FrameGrabber::SWTrigger
     */
    void SWTrigger();

    /**
     * Passthough to ifm3d::FrameGrabber::WaitForFrame
     */
    bool WaitForFrame(const ifm3d::OpenCVBuffer::Ptr& buff,
                      long timeout_millis,
                      bool copy_buff,
                      bool organize);

    /**
     * Resets the internal shared_ptr and creates a new FrameGrabber
     * with the provided arguments. Does this in two distinct steps
     * to force destruction of the object (for the O3X)
     */
    void Reset(ifm3d::Camera::Ptr cam,
               std::uint16_t mask = ifm3d::DEFAULT_SCHEMA_MASK);

  private:
    ifm3d::FrameGrabber::Ptr fg_;
  };

  FrameGrabberWrapper::FrameGrabberWrapper(ifm3d::Camera::Ptr cam,
                                           std::uint16_t mask)
    : fg_(std::make_shared<ifm3d::FrameGrabber>(cam, mask)) {}

  void FrameGrabberWrapper::SWTrigger()
  {
    this->fg_->SWTrigger();
  }

  bool FrameGrabberWrapper::WaitForFrame(
      const ifm3d::OpenCVBuffer::Ptr& buff,
      long timeout_millis,
      bool copy_buff,
      bool organize)
  {
    return this->fg_->WaitForFrame(buff.get(), timeout_millis,
                                   copy_buff, organize);
  }

  void FrameGrabberWrapper::Reset(ifm3d::Camera::Ptr cam, std::uint16_t mask)
  {
    // Two distinct steps (required because O3X only accepts one connection)
    this->fg_.reset();
    this->fg_ = std::make_shared<ifm3d::FrameGrabber>(cam, mask);
  }
}

PYBIND11_MODULE(ifm3dpy, m)
{
  m.doc() = "Bindings for the ifm3d Camera Library";

  // Module metadata
  m.attr("__version__") = std::to_string(IFM3D_VERSION_MAJOR) + "." +
                          std::to_string(IFM3D_VERSION_MINOR) + "." +
                          std::to_string(IFM3D_VERSION_PATCH);
  m.attr("__package__") = "ifm3dpy";

  // Camera defaults
  m.attr("DEFAULT_IP") = ifm3d::DEFAULT_IP;
  m.attr("DEFAULT_XMLRPC_PORT") = ifm3d::DEFAULT_XMLRPC_PORT;
  m.attr("DEFAULT_PASSWORD") = ifm3d::DEFAULT_PASSWORD;

  // Constants used to create "pluggable schema masks"
  m.attr("DEFAULT_SCHEMA_MASK") = ifm3d::DEFAULT_SCHEMA_MASK;
  m.attr("IMG_RDIS") = ifm3d::IMG_RDIS;
  m.attr("IMG_AMP") = ifm3d::IMG_AMP;
  m.attr("IMG_RAMP") = ifm3d::IMG_RAMP;
  m.attr("IMG_CART") = ifm3d::IMG_CART;
  m.attr("IMG_UVEC") = ifm3d::IMG_UVEC;
  m.attr("EXP_TIME") = ifm3d::EXP_TIME;
  m.attr("IMG_GRAY") = ifm3d::IMG_GRAY;
  m.attr("ILLU_TEMP")  = ifm3d::ILLU_TEMP;
  m.attr("INTR_CAL") = ifm3d::INTR_CAL;
  m.attr("INV_INTR_CAL") = ifm3d::INV_INTR_CAL;
  m.attr("JSON_MODEL") = ifm3d::JSON_MODEL;

  // Constants to use for querying supported firmware versions
  m.attr("O3D_TIME_SUPPORT_MAJOR") = ifm3d::O3D_TIME_SUPPORT_MAJOR;
  m.attr("O3D_TIME_SUPPORT_MINOR") = ifm3d::O3D_TIME_SUPPORT_MINOR;
  m.attr("O3D_TIME_SUPPORT_PATCH") = ifm3d::O3D_TIME_SUPPORT_PATCH;

  m.attr("O3D_TMP_PARAMS_SUPPORT_MAJOR") = ifm3d::O3D_TMP_PARAMS_SUPPORT_MAJOR;
  m.attr("O3D_TMP_PARAMS_SUPPORT_MINOR") = ifm3d::O3D_TMP_PARAMS_SUPPORT_MINOR;
  m.attr("O3D_TMP_PARAMS_SUPPORT_PATCH") = ifm3d::O3D_TMP_PARAMS_SUPPORT_PATCH;

  m.attr("O3D_INTRINSIC_PARAM_SUPPORT_MAJOR") =
    ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_MAJOR;
  m.attr("O3D_INTRINSIC_PARAM_SUPPORT_MINOR") =
    ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_MINOR;
  m.attr("O3D_INTRINSIC_PARAM_SUPPORT_PATCH") =
    ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_PATCH;

  m.attr("O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR") =
    ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR;
  m.attr("O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR") =
    ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR;
  m.attr("O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH") =
    ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH;

  py::class_<ifm3d::OpenCVBuffer, ifm3d::OpenCVBuffer::Ptr>(
    m,
    "ImageBuffer",
    R"(
      Class which holds a validated image buffer from the sensor that
      represents a single time-synchronized set of images based on the current
      schema mask set on the active framegrabber.
    )")
    .def(
      py::init<>(),
      R"(
        Constructor
      )")
    .def(
      "extrinsics",
      &ifm3d::OpenCVBuffer::Extrinsics,
      R"(
        Returns a 6-element vector containing the extrinsic
        calibration of the camera. NOTE: This is the extrinsics WRT to the ifm
        optical frame.

        The elements are: tx, ty, tz, rot_x, rot_y, rot_z

        Translation units are mm, rotations are degrees

        Users of this library are highly DISCOURAGED from using the extrinsic
        calibration data stored on the camera itself.
      )")
    .def(
      "intrinsics",
      &ifm3d::OpenCVBuffer::Intrinsics,
      R"(
        Retrieves the intrinsic calibration of the camera

        Returns
        -------
        list[float]
            16-element list containing the intrinsic calibration of the camera
            The elements are:

            Name   Unit          Description
            fx     px            Focal length of the camera in the sensor's x axis direction.
            fy     px            Focal length of the camera in the sensor's y axis direction.
            mx     px            Main point in the sensor's x direction
            my     px            Main point in the sensor's y direction
            alpha  dimensionless Skew parameter
            k1     dimensionless First radial distortion coefficient
            k2     dimensionless Second radial distortion coefficient
            k5     dimensionless Third radial distortion coefficient
            k3     dimensionless First tangential distortion coefficient
            k4     dimensionless Second tangential distortion coefficient
            transX mm            Translation along x-direction in meters.
            transY mm            Translation along y-direction in meters.
            transZ mm            Translation along z-direction in meters.
            rotX   float  degree Rotation along x-axis in radians. Positive values indicate clockwise rotation.
            rotY   float  degree Rotation along y-axis in radians. Positive values indicate clockwise rotation.
            rotZ   float  degree Rotation along z-axis in radians. Positive values indicate clockwise rotation.
      )")
    .def(
      "inverse_intrinsics",
      &ifm3d::OpenCVBuffer::InverseIntrinsics,
      R"(
        Retrieves the inverse intrinsic calibration of the camera. See the
        documentation for ifm3dpy.intrinsics for details on contents.
      )")
    .def(
      "exposure_times",
      &ifm3d::OpenCVBuffer::ExposureTimes,
      R"(
        Returns the exposure times for the current frame.

        Returns
        -------
        list[int]
            A 3-element vector containing the exposure times (usec) for the
            current frame. Unused exposure times are reported as 0.

            If all elements are reported as 0 either the exposure times are not
            configured to be returned back in the data stream from the camera
            or an error in parsing them has occured.
      )")
    .def(
      "timestamp",
      [](const ifm3d::OpenCVBuffer::Ptr& buff)
      {
        // The system clock duration type is different on different platforms,
        // which breaks pybind11 type resolution. We must explicitly cast from
        // ifm3d::TimeStampT to a time_point with a duration that matches
        // the system clock of the target platform.
        auto t =
          std::chrono::time_point_cast<std::chrono::system_clock::duration>(
            buff->TimeStamp());
        return t;
      },
      R"(
        Returns the time stamp of the image data.

        NOTE: To get the timestamp of the confidence data, you
        need to make sure your current pcic schema mask have enabled confidence
        data.

        Returns
        -------
        datetime.datetime
      )")
    .def(
      "illu_temp",
      &ifm3d::OpenCVBuffer::IlluTemp,
      R"(
        Returns the temperature of the illumination unit.

        NOTE: To get the temperature of the illumination unit to the frame, you
        need to make sure your current pcic schema asks for it.

        Returns
        -------
        float
            The temperature of the illumination unit
      )")
    .def(
      "json_model",
      [](const ifm3d::OpenCVBuffer::Ptr& buff)
      {
        // Convert the JSON to a python JSON object using the json module
        py::object json_loads = py::module::import("json").attr("loads");
        return json_loads(buff->JSONModel());
      },
      R"(
        Returns the JSON model of the output of the active application

        NOTE: To get the JSON data for the application running on the device,
        you need to make sure your current pcic schema asks for it by including
        ifm3d::JSON_MODEL in the schema. This will return an empty dict for
        Camera devices like the O3D303, versus ifm Smart Sensors like the 
        O3D302.

        Returns
        -------
        dict
            A JSON encoding of the model
      )")
    .def(
      "organize",
      &ifm3d::OpenCVBuffer::Organize,
      R"(
        This is the interface hook that synchronizes the internally wrapped
        byte buffer with the semantically meaningful image/cloud data
        structures. Within the overall `ifm3d` framework, this function is
        called by the `FrameGrabber` when a complete "frame packet" has been
        recieved. This then parses the bytes and, in-line, will statically
        dispatch to the underly dervied class to populate their image/cloud
        data structures.

        Additionally, this function will populate the extrinsics, exposure
        times, timestamp, and illumination temperature as appropriate and
        subject to the current pcic schema.

        NOTE: This function is called automatically as needed the first time
        frame data are accessed.
      )")
    .def(
      "distance_image",
      [](const ifm3d::OpenCVBuffer::Ptr& buff)
      {
        return ifm3d::image_to_array(buff->DistanceImage());
      },
      R"(
        Retrieves the radial distance image

        Returns
        -------
        numpy.ndarray
            Image organized on the pixel array [rows, cols]
      )")
    .def(
      "unit_vectors",
      [](const ifm3d::OpenCVBuffer::Ptr& buff)
      {
        return ifm3d::image_to_array(buff->UnitVectors());
      },
      R"(
        Retrieves the unit vector image

        Returns
        -------
        numpy.ndarray
            Image organized on the pixel array [rows, cols]
      )")
    .def(
      "gray_image",
      [](const ifm3d::OpenCVBuffer::Ptr& buff)
      {
        return ifm3d::image_to_array(buff->GrayImage());
      },
      R"(
        Retrieves the gray image

        Returns
        -------
        numpy.ndarray
            Image organized on the pixel array [rows, cols]
      )")
    .def(
      "amplitude_image",
      [](const ifm3d::OpenCVBuffer::Ptr& buff)
      {
        return ifm3d::image_to_array(buff->AmplitudeImage());
      },
      R"(
        Retrieves the amplitude image

        Returns
        -------
        numpy.ndarray
            Image organized on the pixel array [rows, cols]
      )")
    .def(
      "raw_amplitude_image",
      [](const ifm3d::OpenCVBuffer::Ptr& buff)
      {
        return ifm3d::image_to_array(buff->RawAmplitudeImage());
      },
      R"(
        Retrieves the raw amplitude image

        Returns
        -------
        numpy.ndarray
            Image organized on the pixel array [rows, cols]
      )")
    .def(
      "confidence_image",
      [](const ifm3d::OpenCVBuffer::Ptr& buff)
      {
        return ifm3d::image_to_array(buff->ConfidenceImage());
      },
      R"(
        Retrieves the confidence image

        Returns
        -------
        numpy.ndarray
            Image organized on the pixel array [rows, cols]
      )")
    .def(
      "xyz_image",
      [](const ifm3d::OpenCVBuffer::Ptr& buff)
      {
        return ifm3d::image_to_array(buff->XYZImage());
      },
      R"(
        Retrieves the xyz image (cartesian point cloud)

        Returns
        -------
        numpy.ndarray nxmx3
            Image organized on the pixel array [rows, cols, chans(x,y,z)]
      )");

  py::class_<ifm3d::FrameGrabberWrapper, ifm3d::FrameGrabberWrapper::Ptr>(
    m,
    "FrameGrabber",
    R"(
      Implements a TCP FrameGrabber connected to a provided Camera
      )")
    .def(
      py::init<ifm3d::Camera::Ptr, std::uint16_t>(),
      py::arg("cam"),
      py::arg("mask") = ifm3d::DEFAULT_SCHEMA_MASK,
      R"(
        Constructor

        Parameters
        ----------
        cam : ifm3dpy.Camera
            The camera instance to grab frames from.

        mask : uint16
            A bitmask encoding the image acquisition schema to stream in from
            the camera.
        )")
    .def(
      "sw_trigger",
      &ifm3d::FrameGrabberWrapper::SWTrigger,
      R"(
        Triggers the camera for image acquisition

        You should be sure to set the `TriggerMode` for your application to
        `SW` in order for this to be effective. This function
        simply does the triggering, data are still received asynchronously via
        `WaitForFrame()`.

        Calling this function when the camera is not in `SW` trigger mode or on
        a device that does not support software-trigger should result in a NOOP
        and no error will be returned (no exceptions thrown). However, we do
        not recommend calling this function in a tight framegrabbing loop when
        you know it is not needed. The "cost" of the NOOP is undefined and
        incurring it is not recommended.
      )")
    .def(
      "wait_for_frame",
      &ifm3d::FrameGrabberWrapper::WaitForFrame,
      py::call_guard<py::gil_scoped_release>(),
      py::arg("buff"),
      py::arg("timeout_millis") = 0,
      py::arg("copy_buff") = false,
      py::arg("organize") = true,
      R"(
        This function is used to grab and parse out time synchronized image
        data from the camera.

        Parameters
        ----------
        buff : ifm3dpy.FrameBuffer
            A FrameBuffer object to update with the latest data from the
            camera.

        timeout_millis : int
            Timeout in millis to wait for new image data from the FrameGrabber.
            If `timeout_millis` is set to 0, this function will block
            indefinitely.

        copy_buff : bool
            Flag indicating whether the framegrabber's internal
            buffer should be copied (O(n)) or swapped
            (O(1)) with the raw bytes of the passed in
            `buff`. You should only flag this as `true` if
            you are planning to use multiple clients with
            a single `FrameGrabber` -- even then, think
            carefully before copying data around.

        organize : bool
            Flag indicating whether or not `Organize` should be called on the
            `ByteBuffer` before returning, or whether it should be deferred
            until the data is accessed.

        Returns
        -------
        bool
            True if a new buffer was acquired w/in "timeout_millis", false
            otherwise.
      )")
  .def(
    "reset",
    &ifm3d::FrameGrabberWrapper::Reset,
    py::arg("cam"),
    py::arg("mask") = ifm3d::DEFAULT_SCHEMA_MASK,
    R"(
      Resets the FrameGrabber with a new camera/bitmask

      Parameters
      ----------
      cam : ifm3dpy.Camera
          The camera instance to grab frames from.

      mask : uint16
          A bitmask encoding the image acquisition schema to stream in from
          the camera.
      )");

  /**
   * Bindings for the Camera module
   */

  py::class_<ifm3d::Camera, ifm3d::Camera::Ptr> camera(
    m, "Camera",
    R"(
      Class for managing an instance of an O3D/O3X Camera
    )");

  // Types

  py::enum_<ifm3d::Camera::boot_mode>(camera, "boot_mode")
    .value("PRODUCTIVE", ifm3d::Camera::boot_mode::PRODUCTIVE)
    .value("RECOVERY", ifm3d::Camera::boot_mode::RECOVERY);

  py::enum_<ifm3d::Camera::operating_mode>(camera, "operating_mode")
    .value("RUN", ifm3d::Camera::operating_mode::RUN)
    .value("EDIT", ifm3d::Camera::operating_mode::EDIT);

  py::enum_<ifm3d::Camera::trigger_mode>(camera, "trigger_mode")
    .value("FREE_RUN", ifm3d::Camera::trigger_mode::FREE_RUN)
    .value("SW", ifm3d::Camera::trigger_mode::SW);

  py::enum_<ifm3d::Camera::import_flags>(camera, "import_flags")
    .value("GLOBAL", ifm3d::Camera::import_flags::GLOBAL)
    .value("NET", ifm3d::Camera::import_flags::NET)
    .value("APPS", ifm3d::Camera::import_flags::APPS);

  py::enum_<ifm3d::Camera::spatial_filter>(camera, "spatial_filter")
    .value("OFF", ifm3d::Camera::spatial_filter::OFF)
    .value("MEDIAN", ifm3d::Camera::spatial_filter::MEDIAN)
    .value("MEAN", ifm3d::Camera::spatial_filter::MEAN)
    .value("BILATERAL", ifm3d::Camera::spatial_filter::BILATERAL);

  py::enum_<ifm3d::Camera::temporal_filter>(camera, "temporal_filter")
    .value("OFF", ifm3d::Camera::temporal_filter::OFF)
    .value("MEAN", ifm3d::Camera::temporal_filter::MEAN)
    .value("ADAPTIVE_EXP", ifm3d::Camera::temporal_filter::ADAPTIVE_EXP);

  py::enum_<ifm3d::Camera::mfilt_mask_size>(camera, "mfilt_mask_size")
    .value("_3x3", ifm3d::Camera::mfilt_mask_size::_3x3)
    .value("_5x5", ifm3d::Camera::mfilt_mask_size::_5x5);

  // Ctor

  camera.def(
    py::init(&ifm3d::Camera::MakeShared),
    R"(
      Constructor

      Parameters
      ----------
      ip : string, optional
          The ip address of the camera. Defaults to 192.168.0.69.

      xmlrpc_port : uint, optional
          The tcp port the sensor's XMLRPC server is listening on. Defaults to
          port 80.

      password : string, optional
          Password required for establishing an "edit session" with the sensor.
          Edit sessions allow for mutating camera parameters and persisting
          those changes. Defaults to '' (no password).
    )",
    py::arg("ip") = ifm3d::DEFAULT_IP,
    py::arg("xmlrpc_port") = ifm3d::DEFAULT_XMLRPC_PORT,
    py::arg("password") = ifm3d::DEFAULT_PASSWORD);

  // Accessors/Mutators

  camera.def_property_readonly(
    "ip",
    &ifm3d::Camera::IP,
    R"(The IP address associated with this Camera instance)");

  camera.def_property_readonly(
    "xmlrpc_port",
    &ifm3d::Camera::XMLRPCPort,
    R"(The XMLRPC port associated with this Camera instance)");

  camera.def_property(
    "password",
    &ifm3d::Camera::Password,
    &ifm3d::Camera::SetPassword,
    R"(The password associated with this Camera instance)");

  camera.def_property_readonly(
    "session_id",
    &ifm3d::Camera::SessionID,
    R"(Retrieves the active session ID)");

  // Member Functions

  camera.def(
    "request_session",
    &ifm3d::Camera::RequestSession,
    R"(
      Requests an edit-mode session with the camera.

      In order to (permanently) mutate parameters on the camera, an edit
      session needs to be established. Only a single edit sesson may be
      established at any one time with the camera (think of it as a global
      mutex on the camera state -- except if you ask for the mutex and it is
      already taken, an exception will be thrown).

      Most typical use-cases for end-users will not involve establishing an
      edit-session with the camera. To mutate camera parameters, the
      `FromJSON` family of functions should be used, which, under-the-hood,
      on the user's behalf, will establish the edit session and gracefully
      close it. There is an exception. For users who plan to modulate imager
      parameters (temporary parameters) on the fly while running the
      framegrabber, managing the session manually is necessary. For this
      reason, we expose this method in the public `Camera` interface.

      NOTE: The session timeout is implicitly set to `ifm3d::MAX_HEARTBEAT`
      after the session has been successfully established.

      Returns
      -------
      str
          The session id issued or accepted by the camera (see
          IFM3D_SESSION_ID environment variable)

      Raises
      ------
      RuntimeError

      @throws ifm3d::error_t if an error is encountered.
    )");

  camera.def(
    "cancel_session",
    (bool (ifm3d::Camera::*)(void)) &ifm3d::Camera::CancelSession,
    R"(
      Explictly stops the current session with the sensor.

      Returns
      -------
      bool
          Indicates success or failure. On failure, check the ifm3d system log
          for details.
    )");

  camera.def(
    "cancel_session",
    (bool (ifm3d::Camera::*)(const std::string&))&ifm3d::Camera::CancelSession,
    py::arg("sid"),
    R"(
      Attempts to cancel a session with a particular session id.

      Parameters
      ----------
      sid : str
          Session ID to cancel.

      Returns
      -------
      bool
          Indicates success or failure. On failure, check the ifm3d system log
          for details.
    )");

  camera.def(
    "heartbeat",
    &ifm3d::Camera::Heartbeat,
    py::arg("hb"),
    R"(
      Sends a heartbeat message and sets the next heartbeat interval

      Heartbeat messages are used to keep a session with the sensor
      alive. This function sends a heartbeat message to the sensor and sets
      when the next heartbeat message is required.

      Parameters
      ----------
      hb : int
          The time (seconds) of when the next heartbeat message will
          be required.

      Returns
      -------
      int
          The current timeout interval in seconds for heartbeat messages

      Raises
      ------
      RuntimeError
    )");

  camera.def(
    "set_temporary_application_parameters",
    &ifm3d::Camera::SetTemporaryApplicationParameters,
    py::arg("params"),
    R"(
      Sets temporary application parameters in run mode.

      The changes are not persistent and are lost when entering edit mode or
      turning the device off. The parameters "ExposureTime" and
      "ExposureTimeRatio" of the imager configuration are supported. All
      additional parameters are ignored (for now). Exposure times are clamped
      to their allowed range, depending on the exposure mode. The user must
      provide the complete set of parameters depending on the exposure mode,
      i.e., "ExposureTime" only for single exposure modes and both
      "ExposureTime" and "ExposureTimeRatio" for double exposure
      modes. Otherwise, behavior is undefined.

      Parameters
      ----------
      params : dict[str, str]
          The parameters to set on the camera.

      Raises
      ------
      RuntimeError
    )");

  camera.def(
    "force_trigger",
    &ifm3d::Camera::ForceTrigger,
    R"(
      Sends a S/W trigger to the camera over XMLRPC.

      The O3X does not S/W trigger over PCIC, so, this function
      has been developed specficially for it. For the O3D, this is a NOOP.
    )");

  camera.def(
    "reboot",
    &ifm3d::Camera::Reboot,
    py::arg("mode") = ifm3d::Camera::boot_mode::PRODUCTIVE,
    R"(
      Reboot the sensor

      Parameters
      ----------
      mode : Camera.boot_mode
          The system mode to boot into upon restart of the sensor

      Raises
      ------
      RuntimeError
    )");

  camera.def(
    "active_application",
    &ifm3d::Camera::ActiveApplication,
    R"(
      Returns the index of the active application.

      A negative number indicates no application is marked as active on the
      sensor.
    )");

  camera.def(
    "device_type",
    &ifm3d::Camera::DeviceType,
    py::arg("use_cached") = true,
    R"(
      Obtains the device type of the connected camera.

      This is a convenience function for extracting out the device type of the
      connected camera. The primary intention of this function is for internal
      usage (i.e., to trigger conditional logic based on the model hardware
      we are talking to) however, it will likely be useful in
      application-level logic as well, so, it is available in the public
      interface.

      Parameters
      ----------
      use_cached : bool
          If set to true, a cached lookup of the device type will be used as
          the return value. If false, it will make a network call to the camera
          to get the "real" device type. The only reason for setting this to
          `false` would be if you expect over the lifetime of your camera
          instance that you will swap out (for example) an O3D for an O3X (or
          vice versa) -- literally, swapping out the network cables while an
          object instance is still alive. If that is not something you are
          worried about, leaving this set to true should result in a signficant
          performance increase.

      Returns
      -------
      str
          Type of device connected
    )");

  camera.def(
    "is_O3X",
    &ifm3d::Camera::IsO3X,
    R"(
      Checks if the device is in the O3X family

      Returns
      -------
      bool
          True for an O3X device
    )");

  camera.def(
    "is_O3D",
    &ifm3d::Camera::IsO3D,
    R"(
      Checks if the device is in the O3D family

      Returns
      -------
      bool
          True for an O3D device
    )");

  camera.def(
    "device_parameter",
    &ifm3d::Camera::DeviceParameter,
    py::arg("key"),
    R"(
      Convenience accessor for extracting a device parameter

      No edit session is created on the camera

      Parameters
      ----------
      key : str
          Name of the parameter to extract

      Returns
      -------
      str
          Value of the requested parameter

      Raises
      ------
      RuntimeError
    )");

  camera.def(
    "trace_logs",
    &ifm3d::Camera::TraceLogs,
    py::arg("count"),
    R"(
      Delivers the trace log from the camera

      A session is not required to call this function.

      Parameters
      ----------
      count : int
          Number of entries to retrieve

      Returns
      -------
      list[str]
          List of strings for each entry in the tracelog
    )");

  camera.def(
    "application_list",
    [](const ifm3d::Camera::Ptr& c)
    {
      // Convert the JSON to a python JSON object using the json module
      py::object json_loads = py::module::import("json").attr("loads");
      return json_loads(c->ApplicationList().dump(2));
    },
    R"(
      Delivers basic information about all applications stored on the device.
      A call to this function does not require establishing a session with the
      camera.

      The returned information is encoded as an array of JSON objects.
      Each object in the array is basically a dictionary with the following
      keys: 'index', 'id', 'name', 'description', 'active'

      Returns
      -------
      dict
          A JSON encoding of the application information

      Raises
      ------
      RuntimeError
    )");

  camera.def(
    "application_types",
    &ifm3d::Camera::ApplicationTypes,
    R"(
      Lists the valid application types supported by the sensor.

      Returns
      -------
      list[str]
          List of strings of the available types of applications supported by
          the sensor. Each element in the list is a string suitable to passing
          to 'CreateApplication'.

      Raises
      ------
      RuntimeError
    )");

  camera.def(
    "imager_types",
    &ifm3d::Camera::ImagerTypes,
    R"(
      Lists the valid imager types supported by the sensor.

      Returns
      -------
      list[str]
          List of strings of the available imager types supported by the sensor

      Raises
      ------
      RuntimeError
    )");

  camera.def(
    "copy_application",
    &ifm3d::Camera::CopyApplication,
    py::arg("idx"),
    R"(
      Creates a new application by copying the configuration of another
      application. The device will generate an ID for the new application and
      put it on a free index.

      Parameters
      ----------
      idx : int
          The index of the application to copy

      Returns
      -------
      int
          Index of the new application

      Raises
      ------
      RuntimeError
    )");

  camera.def(
    "create_application",
    &ifm3d::Camera::CreateApplication,
    py::arg("type") = ifm3d::DEFAULT_APPLICATION_TYPE,
    R"(
      Creates a new application on the camera of the given type.

      To figure out valid `type`s, you should call the
      AvailableApplicationTypes()` method.

      Upon creation of the application, the embedded device will initialize
      all parameters as necessary based on the type. However, based on the
      type, the application may not be in an _activatable_ state. That is, it
      can be created and saved on the device, but it cannot be marked as
      active.

      Parameters
      ----------
      type : str, optional
          The (optional) application type to create. By default,
          it will create a new "Camera" application.

      Returns
      -------
      int
          The index of the new application.
    )");

  camera.def(
    "delete_application",
    &ifm3d::Camera::DeleteApplication,
    py::arg("idx"),
    R"(
      Deletes the application at the specified index from the sensor.

      Parameters
      ----------
      idx : int
          The index of the application to delete

      Raises
      ------
      RuntimeError
    )");

  camera.def(
    "set_current_time",
    &ifm3d::Camera::SetCurrentTime,
    py::arg("epoch_secs") = -1,
    R"(
      Sets the current time on the camera

      Parameters
      ----------
      epoch_secs : int, optional
          Time since the Unix epoch in seconds. A value less than 0 will
          implicity set the time to the current system time.
    )");

  camera.def(
    "factory_reset",
    &ifm3d::Camera::FactoryReset,
    R"(
      Sets the camera configuration back to the state in which it shipped from
      the ifm factory.
    )");

  //
  // @TODO: Not tested; not supported on my o3d303?
  // how to expose this info anyway? numpy?
  //
  camera.def(
    "unit_vectors",
    &ifm3d::Camera::UnitVectors,
    R"(
      For cameras that support fetching the Unit Vectors over XML-RPC, this
      function will return those data as a binary blob.

      Returns
      -------
      list[int]
    )");

  camera.def(
    "export_ifm_config",
    &ifm3d::Camera::ExportIFMConfig,
    R"(
      Exports the entire camera configuration in a format compatible with
      Vision Assistant.

      Returns
      -------
      list[int]
    )");

  camera.def(
    "export_ifm_app",
    &ifm3d::Camera::ExportIFMApp,
    py::arg("idx"),
    R"(
      Export the application at the specified index into a byte array suitable
      for writing to a file. The exported bytes represent the ifm serialization
      of an application.

      This function provides compatibility with tools like IFM's Vision
      Assistant.

      Parameters
      ----------
      idx : int
          The index of the application to export.

      Returns
      -------
      list[int]
          A list of bytes representing the IFM serialization of the
          exported application.

      Raises
      ------
      RuntimeError
    )");

  camera.def(
    "import_ifm_config",
    &ifm3d::Camera::ImportIFMConfig,
    py::arg("bytes"),
    py::arg("flags") = 0,
    R"(
      Imports the entire camera configuration in a format compatible with
      Vision Assistant.

      Parameters
      ----------
      bytes : list[int]
          The camera configuration, serialized in the ifm format

      flags : int
    )");

  camera.def(
    "import_ifm_app",
    &ifm3d::Camera::ImportIFMApp,
    py::arg("bytes"),
    R"(
      Import the IFM-encoded application.

      This function provides compatibility with tools like IFM's Vision
      Assistant. An application configuration exported from VA, can be
      imported using this function.

      Parameters
      ----------
      bytes : list[int]
          The raw bytes from the zip'd JSON file. NOTE: This
          function will base64 encode the data for tranmission
          over XML-RPC.

      Returns
      -------
      int
          The index of the imported application.
    )");

  camera.def(
    "check_minimum_firmware_version",
    &ifm3d::Camera::CheckMinimumFirmwareVersion,
    py::arg("major"),
    py::arg("minor"),
    py::arg("patch"),
    R"(
      Checks for a minimum ifm camera software version

      Parameters
      ----------
      major : int
          Major version of software

      minor : int
          Minor Version of software

      patch : int
          Patch Number of software

      Returns
      -------
      bool
          True if current software version is greater
          or equal to the value passed
    )");

  camera.def(
    "to_json",
    [](const ifm3d::Camera::Ptr& c)
    {
      // Convert the JSON to a python JSON object using the json module
      py::object json_loads = py::module::import("json").attr("loads");
      return json_loads(c->ToJSONStr());
    },
    R"(
      A JSON object containing the state of the camera

      Returns
      -------
      dict
          Camera JSON, compatible with python's json module

      Raises
      ------
      RuntimeError
    )");

  camera.def(
    "from_json",
    [](const ifm3d::Camera::Ptr& c, const py::dict& json)
    {
      // Convert the input JSON to string and load it
      py::object json_dumps = py::module::import("json").attr("dumps");
      c->FromJSONStr(json_dumps(json).cast<std::string>());
    },
    py::arg("json"),
    R"(
      Configures the camera based on the parameter values of the passed in
      JSON. This function is _the_ way to tune the
      camera/application/imager/etc. parameters.

      Parameters
      ----------
      json : dict
          A json object encoding a camera configuration to apply
          to the hardware.

      Raises
      ------
      RuntimeError
          If this raises an exception, you are
          encouraged to check the log file as a best effort is made to be
          as descriptive as possible as to the specific error that has
          occured.
    )");
}
