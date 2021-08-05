/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/camera/camera_o3x.h>

//================================================
// O3XCamera class - the public interface
//================================================

ifm3d::O3XCamera::O3XCamera(const std::string& ip,
                            const std::uint16_t xmlrpc_port,
                            const std::string& password)
  : ifm3d::Camera::Camera(ip, xmlrpc_port, password)
{}

ifm3d::O3XCamera::~O3XCamera() = default;

ifm3d::CameraBase::device_family
ifm3d::O3XCamera::WhoAmI()
{
  return device_family::O3X;
}