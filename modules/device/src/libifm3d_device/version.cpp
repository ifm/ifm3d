/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/device/version.h>

void
ifm3d::version(int* major, int* minor, int* patch)
{
  *major = IFM3D_VERSION_MAJOR;
  *minor = IFM3D_VERSION_MINOR;
  *patch = IFM3D_VERSION_PATCH;
}
