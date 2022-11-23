// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_SCHEMA_H
#define IFM3D_FG_SCHEMA_H

#include <cstdint>
#include <string>
#include <set>
#include <ifm3d/fg/frame_grabber_export.h>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/device/semver.h>

namespace ifm3d
{

  /**
   * Utility function to build an image acquisition result schema from a mask.
   *
   * @param[in] buffer_ids to use to build the schema
   * @return A json-string encoding the schema
   */
  IFM3D_FRAME_GRABBER_EXPORT json
  make_schema(const std::set<ifm3d::buffer_id>& buffer_ids,
              ifm3d::Device::device_family device_type);

  /**
   * Utility function to build a json string, compatible with O3X,
   * for turning on/off image type based on a schema mask
   *
   * @param[in] buffer_ids to use to build the schema
   * @return A json-string comaptible with o3x xmlrpc
   */
  IFM3D_FRAME_GRABBER_EXPORT json
  make_o3x_json_from_mask(const std::set<ifm3d::buffer_id>& chunk_ids);

  /**
   * Utility function makes the schema compatiable with O3R firmware version,
   *
   * @param[in] schema in json
   * @return A json-string comaptible with o3r
   */
  IFM3D_FRAME_GRABBER_EXPORT json
  make_o3r_schema_compatible_with_firmware(const json& o3r_schema,
                                           const ifm3d::SemVer& ver);

  /**
   * Utility function to create a schema mask from a string.
   *
   * The passed in string should contain valid symbolic constants `OR'd`
   * together. For example: IMG_RDIS|IMG_AMP|IMG_RAMP|IMG_CART|IMG_UVEC
   *
   * @param[in] in The string to parse to generate the mask
   * @return The schema encoded by the `in` string.
   */
  IFM3D_FRAME_GRABBER_EXPORT std::uint32_t schema_mask_from_string(
    const std::string& in);

} // end: namespace ifm3d

#endif // IFM3D_FG_SCHEMA_H
