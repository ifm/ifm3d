// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_FG_SCHEMA_H__
#define __IFM3D_FG_SCHEMA_H__

#include <cstdint>
#include <string>
#include <set>
#include <ifm3d/fg/frame_grabber_export.h>
#include <ifm3d/camera/camera_base.h>
#include <ifm3d/fg/frame.h>

namespace ifm3d
{

  /**
   * Utility function to build an image acquisition result schema from a mask.
   *
   * @param[in] image_ids to use to build the schema
   * @return A json-string encoding the schema
   */
  std::string make_schema(const std::set<ifm3d::image_id>& image_ids,
                          ifm3d::CameraBase::device_family device_type);

  /**
   * Utility function to build a json string, compatible with O3X,
   * for turning on/off image type based on a schema mask
   *
   * @param[in] image_ids to use to build the schema
   * @return A json-string comaptible with o3x xmlrpc
   */
  std::string make_o3x_json_from_mask(
    const std::set<ifm3d::image_id>& chunk_ids);

  /**
   * Utility function to create a schema mask from a string.
   *
   * The passed in string should contain valid symbolic constants `OR'd`
   * together. For example: IMG_RDIS|IMG_AMP|IMG_RAMP|IMG_CART|IMG_UVEC
   *
   * @param[in] in The string to parse to generate the mask
   * @return The schema encoded by the `in` string.
   */
  std::uint32_t schema_mask_from_string(const std::string& in);

} // end: namespace ifm3d

#endif // __IFM3D_FG_SCHEMA_H__
