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
#include <memory>
#include <string>
#include <ifm3d/contrib/json.hpp>

using json = nlohmann::json;

namespace ifm3d
{
  extern const std::string DEFAULT_IP;
  extern const std::uint16_t DEFAULT_XMLRPC_PORT;
  extern const std::string DEFAULT_PASSWORD;

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

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

  }; // end: class Camera

} // end: namespace ifm3d

#endif // __IFM3D_CAMERA_CAMERA_H__
