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
#include <ifm3d/fg/frame_grabber_export.h>

namespace ifm3d
{
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t DEFAULT_SCHEMA_MASK;

  // Constants used to create "pluggable schema masks"
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t IMG_RDIS;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t IMG_AMP;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t IMG_RAMP;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t IMG_CART;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t IMG_UVEC;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t EXP_TIME;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t IMG_GRAY;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t ILLU_TEMP;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t INTR_CAL;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t INV_INTR_CAL;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t JSON_MODEL;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::uint16_t IMG_DIS_NOISE;
  /**
   * Utility function to build an image acquisition result schema from a mask.
   *
   * @param[in] mask The mask to use to build the schema
   * @return A json-string encoding the schema
   */
  std::string make_schema(std::uint16_t mask);

  /**
   * Utility function to build a json string, compatible with O3X,
   * for turning on/off image type based on a schema mask
   *
   * @param[in] mask The mask to use to build the schema
   * @return A json-string comaptible with o3x xmlrpc
   */
  std::string make_o3x_json_from_mask(std::uint16_t mask);

  /**
   * Utility function to create a schema mask from a string.
   *
   * The passed in string should contain valid symbolic constants `OR'd`
   * together. For example: IMG_RDIS|IMG_AMP|IMG_RAMP|IMG_CART|IMG_UVEC
   *
   * @param[in] in The string to parse to generate the mask
   * @return The schema encoded by the `in` string.
   */
  std::uint16_t schema_mask_from_string(const std::string& in);

} // end: namespace ifm3d

#endif // __IFM3D_FG_SCHEMA_H__
