/*
 * Copyright 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ifm3d-dist_image_info_test_data.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <gtest/gtest.h>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/distance_image_info.h>
#include <ifm3d/fg/organizer_utils.h>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace ifm3d
{
  constexpr auto INVALID_IDX = std::numeric_limits<std::size_t>::max();
  constexpr auto FLOAT_DATA_SIZE = sizeof(float);
  constexpr auto UINT32_DATA_SIZE = sizeof(std::uint32_t);
  constexpr auto UINT16_DATA_SIZE = sizeof(std::uint16_t);

  namespace
  {

    std::size_t
    get_chunk_index(const std::vector<std::uint8_t>& buff,
                    ifm3d::image_chunk chunk_type,
                    std::size_t start_idx = ifm3d::IMG_BUFF_START)
    {
      std::size_t idx = start_idx; // start of first chunk
      std::size_t const size = buff.size() - 6;

      while (idx < size)
        {
          if (static_cast<std::uint32_t>(chunk_type) ==
              ifm3d::mkval<std::uint32_t>(buff.data() + idx))
            {
              return idx;
            }

          // move to the beginning of the next chunk
          auto const incr = ifm3d::mkval<std::uint32_t>(buff.data() + idx + 4);
          if (incr <= 0)
            {
              LOG_WARNING("Next chunk is supposedly {} bytes from the current "
                          "one ... failing!",
                          incr);
              break;
            }
          idx += incr;
        }

      return INVALID_IDX;
    }

    ifm3d::DistanceImageInfoPtr
    read_o3r_byte_buffer_from_file(const std::string& s_file,
                                   std::uint32_t width = 0,
                                   std::uint32_t height = 0)
    {
      std::vector<std::uint8_t> file_buffer;
      std::ifstream in_file(s_file, std::ios_base::binary);
      EXPECT_TRUE(in_file.is_open());
      std::istreambuf_iterator<char> const iter(in_file);
      std::copy(iter,
                std::istreambuf_iterator<char>(),
                std::back_inserter(file_buffer));
      in_file.close();

      auto didx =
        ifm3d::get_chunk_index(file_buffer,
                               ifm3d::image_chunk::RADIAL_DISTANCE_IMAGE);
      auto aidx =
        ifm3d::get_chunk_index(file_buffer,
                               ifm3d::image_chunk::NORM_AMPLITUDE_IMAGE);
      auto distimageidx =
        ifm3d::get_chunk_index(file_buffer, ifm3d::image_chunk::TOF_INFO);

      // get the image dimensions
      if (!width)
        {
          width = ifm3d::mkval<std::uint32_t>(file_buffer.data() + didx + 16);
        }
      if (!height)
        {
          height = ifm3d::mkval<std::uint32_t>(file_buffer.data() + didx + 20);
        }

      std::cout << "file buffer size: " << file_buffer.size()
                << " distancd image idx: " << distimageidx << '\n';
      std::cout << " - with: " << width << " height: " << height << '\n';

      if (distimageidx == ifm3d::INVALID_IDX || didx == ifm3d::INVALID_IDX ||
          aidx == ifm3d::INVALID_IDX)
        {
          return {};
        }

      return ifm3d::create_distance_image_info(file_buffer,
                                               distimageidx,
                                               didx,
                                               aidx,
                                               width,
                                               height);
    }
  }
};

TEST(DistanceImageInfo, AmplitudeImageConversion)
{
  auto npts = IMAGE_WIDTH * IMAGE_HEIGHT;
  EXPECT_EQ(DISTANCE_BUFFER.size(), npts);
  EXPECT_EQ(AMPLITUDE_BUFFER.size(), npts);
  EXPECT_EQ(EXTRINSIC_OPTIC_TO_USER.size(), 6);
  EXPECT_EQ(sizeof(INTR_CALIBRATION.model_parameters) /
              sizeof(INTR_CALIBRATION.model_parameters[0]),
            32);

  std::vector<float> const amp_norm_fctrs;
  ifm3d::IntrinsicCalibration const inv_intr_calib{};
  ifm3d::DistanceImageInfo distance_image_info{DISTANCE_RESOLUTION,
                                               AMPLITUDE_RESOLUTION,
                                               {},
                                               EXTRINSIC_OPTIC_TO_USER,
                                               INTR_CALIBRATION,
                                               {},
                                               DISTANCE_BUFFER,
                                               AMPLITUDE_BUFFER,
                                               {},
                                               {},
                                               IMAGE_WIDTH,
                                               IMAGE_HEIGHT};

  auto amplitude_vector = distance_image_info.GetAmplitudeVector();
  EXPECT_EQ(amplitude_vector.size(), npts * ifm3d::FLOAT_DATA_SIZE);
  EXPECT_EQ(AMPLITUDE_BUFFER_CALC.size(), npts);

  for (int i = 0; i < AMPLITUDE_BUFFER_CALC.size(); ++i)
    {
      EXPECT_NEAR(AMPLITUDE_BUFFER_CALC[i],
                  ifm3d::mkval<float>(amplitude_vector.data() +
                                      (i * ifm3d::FLOAT_DATA_SIZE)),
                  1e-4);
    }
}

TEST(DistanceImageInfo, DistanceImageConversion)
{
  auto npts = IMAGE_WIDTH * IMAGE_HEIGHT;
  EXPECT_EQ(DISTANCE_BUFFER.size(), npts);
  EXPECT_EQ(AMPLITUDE_BUFFER.size(), npts);
  EXPECT_EQ(EXTRINSIC_OPTIC_TO_USER.size(), 6);
  EXPECT_EQ(sizeof(INTR_CALIBRATION.model_parameters) /
              sizeof(INTR_CALIBRATION.model_parameters[0]),
            32);

  std::vector<float> const amp_norm_fctrs;
  ifm3d::IntrinsicCalibration const inv_intr_calib{};
  ifm3d::DistanceImageInfo distance_image_info(DISTANCE_RESOLUTION,
                                               AMPLITUDE_RESOLUTION,
                                               {},
                                               EXTRINSIC_OPTIC_TO_USER,
                                               INTR_CALIBRATION,
                                               {},
                                               DISTANCE_BUFFER,
                                               AMPLITUDE_BUFFER,
                                               {},
                                               {},
                                               IMAGE_WIDTH,
                                               IMAGE_HEIGHT);

  auto xyzd_vector = distance_image_info.GetXyzdVector();
  EXPECT_EQ(xyzd_vector.size(),
            static_cast<std::size_t>(npts) * 4 * ifm3d::FLOAT_DATA_SIZE);

  EXPECT_EQ(X_CALC.size(), npts);
  for (int i = 0; i < X_CALC.size(); ++i)
    {
      EXPECT_NEAR(
        X_CALC[i],
        ifm3d::mkval<float>(xyzd_vector.data() + (ifm3d::FLOAT_DATA_SIZE * i)),
        1e-10);
    }

  auto y_vector_offset = (npts * ifm3d::FLOAT_DATA_SIZE);
  EXPECT_EQ(Y_CALC.size(), npts);

  for (int i = 0; i < Y_CALC.size(); ++i)
    {
      EXPECT_NEAR(Y_CALC[i],
                  ifm3d::mkval<float>(xyzd_vector.data() + y_vector_offset +
                                      (ifm3d::FLOAT_DATA_SIZE * i)),
                  1e-10);
    }

  auto z_vector_offset =
    (2 * static_cast<std::size_t>(npts) * ifm3d::FLOAT_DATA_SIZE);
  EXPECT_EQ(Z_CALC.size(), npts);

  for (int i = 0; i < Z_CALC.size(); ++i)
    {
      EXPECT_NEAR(Z_CALC[i],
                  ifm3d::mkval<float>(xyzd_vector.data() + z_vector_offset +
                                      (ifm3d::FLOAT_DATA_SIZE * i)),
                  1e-10);
    }

  auto dist_vector_offset =
    (3 * static_cast<std::size_t>(npts) * ifm3d::FLOAT_DATA_SIZE);
  EXPECT_EQ(DISTANCE_BUFFER_CALC.size(), npts);

  for (int i = 0; i < DISTANCE_BUFFER_CALC.size(); ++i)
    {
      EXPECT_NEAR(DISTANCE_BUFFER_CALC[i],
                  ifm3d::mkval<float>(xyzd_vector.data() + dist_vector_offset +
                                      (ifm3d::FLOAT_DATA_SIZE * i)),
                  1e-10);
    }
}

TEST(DistanceImageInfo, DISABLED_InvalidDistImageIndex)
{
  EXPECT_EQ(ifm3d::create_distance_image_info({},
                                              ifm3d::INVALID_IDX,
                                              ifm3d::INVALID_IDX,
                                              ifm3d::INVALID_IDX,
                                              0,
                                              0),
            nullptr);
}

TEST(DistanceImageInfo, InvalidDistImageSize)
{
  // image size to big
  auto distance_image_info =
    ifm3d::read_o3r_byte_buffer_from_file(BUFFER_DATA_FILE, 500, 100);
  EXPECT_NE(distance_image_info, nullptr);
  EXPECT_EQ(distance_image_info->GetAmplitudeVector().size(), 0);
  EXPECT_EQ(distance_image_info->GetXyzdVector().size(), 0);
}

TEST(DistanceImageInfo, AmplitudeImageConversionFromBuffer)
{
  auto distance_image_info =
    ifm3d::read_o3r_byte_buffer_from_file(BUFFER_DATA_FILE);
  EXPECT_NE(distance_image_info, nullptr);
  auto npts = distance_image_info->GetNpts();
  auto amplitude_vector = distance_image_info->GetAmplitudeVector();

  EXPECT_EQ(amplitude_vector.size(), npts * ifm3d::FLOAT_DATA_SIZE);
  for (int i = 0; i < AMPLITUDE.size(); ++i)
    {
      EXPECT_NEAR(AMPLITUDE[i],
                  ifm3d::mkval<float>(amplitude_vector.data() +
                                      (i * ifm3d::FLOAT_DATA_SIZE)),
                  1e-6);
    }
}

TEST(DistanceImageInfo, DistanceImageConversionFromBuffer)
{
  auto distance_image_info =
    ifm3d::read_o3r_byte_buffer_from_file(BUFFER_DATA_FILE);
  EXPECT_NE(distance_image_info, nullptr);
  auto npts = distance_image_info->GetNpts();
  auto xyzd_vector = distance_image_info->GetXyzdVector();

  EXPECT_EQ(xyzd_vector.size(),
            static_cast<std::size_t>(npts) * 4 * ifm3d::FLOAT_DATA_SIZE);

  auto x_offset = COMPARE_BUFFER_DATA_OFFSET * ifm3d::FLOAT_DATA_SIZE;
  for (int i = 0; i < X_VECTOR.size(); ++i)
    {
      EXPECT_NEAR(X_VECTOR[i],
                  ifm3d::mkval<float>(xyzd_vector.data() + x_offset +
                                      (i * ifm3d::FLOAT_DATA_SIZE)),
                  1e-6);
    }

  auto y_offset = (npts * ifm3d::FLOAT_DATA_SIZE) +
                  (COMPARE_BUFFER_DATA_OFFSET * ifm3d::FLOAT_DATA_SIZE);
  for (int i = 0; i < Y_VECTOR.size(); ++i)
    {
      EXPECT_NEAR(Y_VECTOR[i],
                  ifm3d::mkval<float>(xyzd_vector.data() + y_offset +
                                      (i * ifm3d::FLOAT_DATA_SIZE)),
                  1e-6);
    }

  auto z_offset =
    (2 * static_cast<std::size_t>(npts) * ifm3d::FLOAT_DATA_SIZE) +
    (COMPARE_BUFFER_DATA_OFFSET * ifm3d::FLOAT_DATA_SIZE);
  for (int i = 0; i < Z_VECTOR.size(); ++i)
    {
      EXPECT_NEAR(Z_VECTOR[i],
                  ifm3d::mkval<float>(xyzd_vector.data() + z_offset +
                                      (i * ifm3d::FLOAT_DATA_SIZE)),
                  1e-6);
    }
}
