/*
 * Copyright 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include <iostream>
#include <memory>
#include <fstream>
#include <iterator>
#include <vector>
#include <set>
#include <gtest/gtest.h>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/schema.h>

namespace ifm3d
{
  bool
  check_for_id(const json& elements, std::string id)
  {
    for (auto& element : elements)
      {
        if (element["id"] == id)
          {
            return true;
          }
      }
    return false;
  }
};

TEST(Schema, MakeSchema_O3R)
{
  std::set<ifm3d::buffer_id> buffer_ids = {
    ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE,
    ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE};

  EXPECT_NO_THROW(
    ifm3d::make_schema(buffer_ids, ifm3d::Device::device_family::O3R));

  auto schema =
    ifm3d::make_schema(buffer_ids, ifm3d::Device::device_family::O3R);

  EXPECT_TRUE(schema["elements"].is_array());
  // 2 for buffer_ids and 2 for start and stop
  EXPECT_TRUE(schema["elements"].size() == 2 + 2);
}

TEST(Schema, o3r_firmware_compatibility_rgb_info)
{

  auto check_buffer_id_and_id = [&](ifm3d::buffer_id buffer_id) {
    std::set<ifm3d::buffer_id> buffer_ids = {buffer_id};

    auto schema =
      ifm3d::make_schema(buffer_ids, ifm3d::Device::device_family::O3R);

    EXPECT_TRUE(ifm3d::check_for_id(schema["elements"], "RGB_INFO"));

    auto schema_firware_101 =
      ifm3d::make_o3r_schema_compatible_with_firmware(schema,
                                                      ifm3d::SemVer(1, 0, 1));

    EXPECT_TRUE(
      ifm3d::check_for_id(schema_firware_101["elements"], "RGB_INFO"));

    auto schema_firware_above_101 =
      ifm3d::make_o3r_schema_compatible_with_firmware(schema,
                                                      ifm3d::SemVer(1, 1, 0));

    EXPECT_TRUE(
      ifm3d::check_for_id(schema_firware_above_101["elements"], "RGB_INFO"));

    auto schema_firware_below_101 =
      ifm3d::make_o3r_schema_compatible_with_firmware(schema,
                                                      ifm3d::SemVer(0, 16, 0));

    EXPECT_TRUE(ifm3d::check_for_id(schema_firware_below_101["elements"],
                                    "O3R_RGB_IMAGE_INFO"));
  };

  check_buffer_id_and_id(ifm3d::buffer_id::RGB_INFO);
}

TEST(Schema, o3r_firmware_compatibility_tof_info)
{
  auto check_buffer_id_and_id = [&](ifm3d::buffer_id buffer_id) {
    std::set<ifm3d::buffer_id> buffer_ids = {buffer_id};

    auto schema =
      ifm3d::make_schema(buffer_ids, ifm3d::Device::device_family::O3R);

    EXPECT_TRUE(ifm3d::check_for_id(schema["elements"], "TOF_INFO"));

    auto schema_firware_101 =
      ifm3d::make_o3r_schema_compatible_with_firmware(schema,
                                                      ifm3d::SemVer(1, 0, 1));

    EXPECT_TRUE(
      ifm3d::check_for_id(schema_firware_101["elements"], "TOF_INFO"));

    auto schema_firware_above_101 =
      ifm3d::make_o3r_schema_compatible_with_firmware(schema,
                                                      ifm3d::SemVer(1, 1, 0));

    EXPECT_TRUE(
      ifm3d::check_for_id(schema_firware_above_101["elements"], "TOF_INFO"));

    auto schema_firware_below_101 =
      ifm3d::make_o3r_schema_compatible_with_firmware(schema,
                                                      ifm3d::SemVer(0, 16, 0));

    EXPECT_TRUE(
      ifm3d::check_for_id(schema_firware_below_101["elements"], "TOF_INFO"));
  };

  check_buffer_id_and_id(ifm3d::buffer_id::TOF_INFO);
}