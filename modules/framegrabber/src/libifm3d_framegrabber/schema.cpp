/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/fg/schema.h>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>
#include <set>
#include <ifm3d/device/util.h>
#include <ifm3d/device/err.h>
#include <ifm3d/device/json.hpp>

const ifm3d::SemVer O3R_SCHEMA_FIRMWARE_COMPATIBILITY_CHECK_VERSION =
  ifm3d::SemVer(1, 0, 1);

const std::map<ifm3d::buffer_id, const ifm3d::json> o3d_schema_map{
  {ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
   {{"type", "blob"}, {"id", "distance_image"}}},
  {ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE,
   {{"type", "blob"}, {"id", "normalized_amplitude_image"}}},
  {ifm3d::buffer_id::AMPLITUDE_IMAGE,
   {{"type", "blob"}, {"id", "amplitude_image"}}},
  {ifm3d::buffer_id::GRAYSCALE_IMAGE,
   {{"type", "blob"}, {"id", "grayscale_image"}}},
  {ifm3d::buffer_id::CARTESIAN_X_COMPONENT,
   {{"type", "blob"}, {"id", "x_image"}}},
  {ifm3d::buffer_id::CARTESIAN_Y_COMPONENT,
   {{"type", "blob"}, {"id", "y_image"}}},
  {ifm3d::buffer_id::CARTESIAN_Z_COMPONENT,
   {{"type", "blob"}, {"id", "z_image"}}},
  {ifm3d::buffer_id::UNIT_VECTOR_ALL,
   {{"type", "blob"}, {"id", "all_unit_vector_matrices"}}},
  {ifm3d::buffer_id::INTRINSIC_CALIB,
   {{"type", "blob"}, {"id", "intrinsic_calibration"}}},
  {ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION,
   {{"type", "blob"}, {"id", "inverse_intrinsic_calibration"}}},
  {ifm3d::buffer_id::JSON_MODEL, {{"type", "blob"}, {"id", "json_model"}}},
  {ifm3d::buffer_id::CONFIDENCE_IMAGE,
   {{"type", "blob"}, {"id", "confidence_image"}}},
  {ifm3d::buffer_id::EXTRINSIC_CALIB,
   {{"type", "blob"}, {"id", "extrinsic_calibration"}}},
  {ifm3d::buffer_id::EXPOSURE_TIME,
   ifm3d::json::array(
     {{{"type", "string"}, {"id", "exposure_times"}, {"value", "extime"}},
      {{"type", "uint32"},
       {"id", "exposure_time_1"},
       {"format", {{"dataencoding", "binary"}, {"order", "little"}}}},
      {{"type", "uint32"},
       {"id", "exposure_time_2"},
       {"format", {{"dataencoding", "binary"}, {"order", "little"}}}},
      {{"type", "uint32"},
       {"id", "exposure_time_3"},
       {"format", {{"dataencoding", "binary"}, {"order", "little"}}}}})},
  {ifm3d::buffer_id::ILLUMINATION_TEMP,
   ifm3d::json::array(
     {{{"type", "string"}, {"id", "temp_illu"}, {"value", "temp_illu"}},
      {{"type", "float32"},
       {"id", "temp_illu"},
       {"format", {{"dataencoding", "binary"}, {"order", "little"}}}}})},
};

// TODO : update this for O3R specific Data.
const std::map<ifm3d::buffer_id, const ifm3d::json> o3r_schema_map{
  {ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
   {{"type", "blob"}, {"id", "RADIAL_DISTANCE_COMPRESSED"}}},
  {ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE,
   {{"type", "blob"}, {"id", "AMPLITUDE_COMPRESSED"}}},
  {ifm3d::buffer_id::ALGO_DEBUG, {{"type", "blob"}, {"id", "ALGO_DEBUG"}}},
  {ifm3d::buffer_id::REFLECTIVITY, {{"type", "blob"}, {"id", "REFLECTIVITY"}}},
  {ifm3d::buffer_id::INTRINSIC_CALIB,
   {{"type", "blob"}, {"id", "intrinsic_calibration"}}},
  {ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION,
   {{"type", "blob"}, {"id", "inverse_intrinsic_calibration"}}},
  {ifm3d::buffer_id::RADIAL_DISTANCE_NOISE,
   {{"type", "blob"}, {"id", "RADIAL_DISTANCE_NOISE"}}},
  {ifm3d::buffer_id::TOF_INFO, {{"type", "blob"}, {"id", "TOF_INFO"}}},
  {ifm3d::buffer_id::JPEG_IMAGE, {{"type", "blob"}, {"id", "JPEG_IMAGE"}}},
  {ifm3d::buffer_id::CONFIDENCE_IMAGE,
   {{"type", "blob"}, {"id", "CONFIDENCE"}}},
  {ifm3d::buffer_id::RGB_INFO, {{"type", "blob"}, {"id", "RGB_INFO"}}},
  {ifm3d::buffer_id::O3R_ODS_INFO, {{"type", "blob"}, {"id", "O3R_ODS_INFO"}}},
  {ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID,
   {{"type", "blob"}, {"id", "O3R_ODS_OCCUPANCY_GRID"}}},
};

ifm3d::json
ifm3d::make_o3x_json_from_mask(const std::set<ifm3d::buffer_id>& buffer_ids)
{
  std::map<size_t, std::string> bool_to_string{{0, "false"}, {1, "true"}};

  ifm3d::json schema = {{"Apps", ifm3d::json::array({{{"Index", "1"}}})}};

  auto& app_json_pointer = schema["/Apps/0"_json_pointer];

  app_json_pointer["OutputDistanceImage"] =
    bool_to_string[buffer_ids.count(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE)];
  app_json_pointer["OutputAmplitudeImage"] =
    bool_to_string[buffer_ids.count(ifm3d::buffer_id::AMPLITUDE_IMAGE)];
  app_json_pointer["OutputGrayscaleImage"] =
    bool_to_string[buffer_ids.count(ifm3d::buffer_id::GRAYSCALE_IMAGE)];
  app_json_pointer["OutputXYZImage"] =
    bool_to_string[buffer_ids.count(ifm3d::buffer_id::XYZ) ||
                   buffer_ids.count(ifm3d::buffer_id::CARTESIAN_ALL) ||
                   buffer_ids.count(ifm3d::buffer_id::CARTESIAN_X_COMPONENT) ||
                   buffer_ids.count(ifm3d::buffer_id::CARTESIAN_Y_COMPONENT) ||
                   buffer_ids.count(ifm3d::buffer_id::CARTESIAN_Z_COMPONENT)];
  app_json_pointer["OutputDistanceNoiseImage"] =
    bool_to_string[buffer_ids.count(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE)];
  app_json_pointer["OutputConfidenceImage"] =
    bool_to_string[buffer_ids.count(ifm3d::buffer_id::CONFIDENCE_IMAGE)];

  return schema;
}

ifm3d::json
ifm3d::make_schema(const std::set<ifm3d::buffer_id>& buffer_ids,
                   ifm3d::Device::device_family device_type)
{
  if (device_type == ifm3d::Device::device_family::O3X)
    {
      return make_o3x_json_from_mask(buffer_ids);
    }

  auto check_for_device_support =
    [](const ifm3d::buffer_id buffer_id,
       const std::map<ifm3d::buffer_id, const ifm3d::json>& schema_map)
    -> ifm3d::json {
    if (schema_map.find(buffer_id) != schema_map.end())
      {
        return schema_map.at(buffer_id);
      }
    else
      {
        // TODO: should we throw if schema generation fail due to non supported
        // chunk_id by device ? throw
        // ifm3d::error_t(IFM3D_UNSUPPORTED_SCHEMA_ON_DEVICE);
      }
    return {};
  };

  ifm3d::json schema = {
    {"layouter", "flexible"},
    {"format", {{"dataencoding", "ascii"}}},
    {"elements",
     ifm3d::json::array(
       {{{"type", "string"}, {"value", "star"}, {"id", "start_string"}}})}};

  auto& elements = schema["/elements"_json_pointer];

  auto schema_generator =
    [&](const std::map<ifm3d::buffer_id, const ifm3d::json>& schema_map) {
      for (const auto chunk_id : buffer_ids)
        {
          auto json_schema_for_id =
            check_for_device_support(chunk_id, schema_map);

          if (json_schema_for_id.is_null())
            {
              continue;
            }

          if (json_schema_for_id.is_array())
            {
              for (const auto& val : json_schema_for_id)
                {
                  elements.push_back(val);
                }
            }
          else
            {
              elements.push_back(json_schema_for_id);
            }
        }
    };

  if (device_type == ifm3d::Device::device_family::O3D)
    {
      schema_generator(o3d_schema_map);
    }
  else if (device_type == ifm3d::Device::device_family::O3R)
    {
      schema_generator(o3r_schema_map);
      // TODO how to enable exposure time for O3R ?
    }

  // Add stop to the schema
  elements.push_back(
    {{"type", "string"}, {"value", "stop"}, {"id", "end_string"}});
  return schema;
}

ifm3d::json
ifm3d::make_o3r_schema_compatible_with_firmware(const json& o3r_schema,
                                                const ifm3d::SemVer& ver)
{
  auto schema = o3r_schema;
  if (ver < O3R_SCHEMA_FIRMWARE_COMPATIBILITY_CHECK_VERSION)
    {
      // find and change id RGB_INFO to O3R_RGB_IMAGE_INFO
      auto& elements = schema["elements"];

      for (auto& item : elements)
        {
          if (item["id"] == "RGB_INFO")
            {
              item["id"] = "O3R_RGB_IMAGE_INFO";
            }
        }
    }
  return schema;
}