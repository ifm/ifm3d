/*
 * Copyright 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include <iostream>
#include <memory>
#include <fstream>
#include <iterator>
#include <vector>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <ifm3d/fg/organizer_utils.h>
#include <ifm3d/fg/frame_grabber.h>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/distance_image_info.h>
#include "ifm3d-dist_image_info_test_data.h"

namespace ifm3d
{
  constexpr auto INVALID_IDX = std::numeric_limits<std::size_t>::max();
  constexpr auto FLOAT_DATA_SIZE = sizeof(float);
  constexpr auto UINT32_DATA_SIZE = sizeof(std::uint32_t);
  constexpr auto UINT16_DATA_SIZE = sizeof(std::uint16_t);

  std::size_t
  get_chunk_index(const std::vector<std::uint8_t>& buff,
                  ifm3d::image_chunk chunk_type,
                  std::size_t start_idx = ifm3d::IMG_BUFF_START)
  {
    std::size_t idx = start_idx; // start of first chunk
    std::size_t size = buff.size() - 6;

    while (idx < size)
      {
        if (static_cast<std::uint32_t>(chunk_type) ==
            ifm3d::mkval<std::uint32_t>(buff.data() + idx))
          {
            return idx;
          }

        // move to the beginning of the next chunk
        std::uint32_t incr =
          ifm3d::mkval<std::uint32_t>(buff.data() + idx + 4);
        if (incr <= 0)
          {
            LOG(WARNING) << "Next chunk is supposedly " << incr
                         << " bytes from the current one ... failing!";
            break;
          }
        idx += incr;
      }

    return INVALID_IDX;
  }
};

ifm3d::DistanceImageInfoPtr
readO3RByteBufferFromFile(std::string sFile,
                          std::uint32_t width = 0,
                          std::uint32_t height = 0)
{
  std::vector<std::uint8_t> fileBuffer;
  std::ifstream inFile(sFile, std::ios_base::binary);
  EXPECT_TRUE(inFile.is_open());
  std::istreambuf_iterator<char> iter(inFile);
  std::copy(iter,
            std::istreambuf_iterator<char>(),
            std::back_inserter(fileBuffer));
  inFile.close();

  auto didx =
    ifm3d::get_chunk_index(fileBuffer,
                           ifm3d::image_chunk::RADIAL_DISTANCE_IMAGE);
  auto aidx = ifm3d::get_chunk_index(fileBuffer,
                                     ifm3d::image_chunk::NORM_AMPLITUDE_IMAGE);
  auto distimageidx =
    ifm3d::get_chunk_index(fileBuffer, ifm3d::image_chunk::TOF_INFO);

  // get the image dimensions
  if (!width)
    {
      width = ifm3d::mkval<std::uint32_t>(fileBuffer.data() + didx + 16);
    }
  if (!height)
    {
      height = ifm3d::mkval<std::uint32_t>(fileBuffer.data() + didx + 20);
    }

  std::cout << "file buffer size: " << fileBuffer.size()
            << " distancd image idx: " << distimageidx << std::endl;
  std::cout << " - with: " << width << " height: " << height << std::endl;

  if (distimageidx == ifm3d::INVALID_IDX || didx == ifm3d::INVALID_IDX ||
      aidx == ifm3d::INVALID_IDX)
    {
      return {};
    }

  return ifm3d::CreateDistanceImageInfo(fileBuffer,
                                        distimageidx,
                                        didx,
                                        aidx,
                                        width,
                                        height);
}

TEST(DistanceImageInfo, AmplitudeImageConversion)
{
  auto npts = image_width * image_height;
  EXPECT_EQ(distance_buffer.size(), npts);
  EXPECT_EQ(amplitude_buffer.size(), npts);
  EXPECT_EQ(extrinsic_optic_to_user.size(), 6);
  EXPECT_EQ(sizeof(intr_calibration.model_parameters) /
              sizeof(intr_calibration.model_parameters[0]),
            32);

  std::vector<float> amp_norm_fctrs;
  ifm3d::IntrinsicCalibration inv_intr_calib;
  ifm3d::DistanceImageInfo distanceImageInfo{distance_resolution,
                                             amplitude_resolution,
                                             {},
                                             extrinsic_optic_to_user,
                                             intr_calibration,
                                             {},
                                             distance_buffer,
                                             amplitude_buffer,
                                             {},
                                             {},
                                             image_width,
                                             image_height};

  auto amplitude_vector = distanceImageInfo.getAmplitudeVector();
  EXPECT_EQ(amplitude_vector.size(), npts * ifm3d::FLOAT_DATA_SIZE);
  EXPECT_EQ(amplitude_buffer_calc.size(), npts);

  for (int i = 0; i < amplitude_buffer_calc.size(); ++i)
    {
      EXPECT_NEAR(amplitude_buffer_calc[i],
                  ifm3d::mkval<float>(amplitude_vector.data() +
                                      i * ifm3d::FLOAT_DATA_SIZE),
                  1e-4);
    }
}

TEST(DistanceImageInfo, DistanceImageConversion)
{
  auto npts = image_width * image_height;
  EXPECT_EQ(distance_buffer.size(), npts);
  EXPECT_EQ(amplitude_buffer.size(), npts);
  EXPECT_EQ(extrinsic_optic_to_user.size(), 6);
  EXPECT_EQ(sizeof(intr_calibration.model_parameters) /
              sizeof(intr_calibration.model_parameters[0]),
            32);

  std::vector<float> amp_norm_fctrs;
  ifm3d::IntrinsicCalibration inv_intr_calib;
  ifm3d::DistanceImageInfo distanceImageInfo(distance_resolution,
                                             amplitude_resolution,
                                             {},
                                             extrinsic_optic_to_user,
                                             intr_calibration,
                                             {},
                                             distance_buffer,
                                             amplitude_buffer,
                                             {},
                                             {},
                                             image_width,
                                             image_height);

  auto xyzd_vector = distanceImageInfo.getXYZDVector();
  EXPECT_EQ(xyzd_vector.size(), npts * 4 * ifm3d::FLOAT_DATA_SIZE);

  EXPECT_EQ(x_calc.size(), npts);
  for (int i = 0; i < x_calc.size(); ++i)
    {
      EXPECT_NEAR(
        x_calc[i],
        ifm3d::mkval<float>(xyzd_vector.data() + (ifm3d::FLOAT_DATA_SIZE * i)),
        1e-10);
    }

  auto y_vector_offset = (npts * ifm3d::FLOAT_DATA_SIZE);
  EXPECT_EQ(y_calc.size(), npts);

  for (int i = 0; i < y_calc.size(); ++i)
    {
      EXPECT_NEAR(y_calc[i],
                  ifm3d::mkval<float>(xyzd_vector.data() + y_vector_offset +
                                      (ifm3d::FLOAT_DATA_SIZE * i)),
                  1e-10);
    }

  auto z_vector_offset = (2 * npts * ifm3d::FLOAT_DATA_SIZE);
  EXPECT_EQ(z_calc.size(), npts);

  for (int i = 0; i < z_calc.size(); ++i)
    {
      EXPECT_NEAR(z_calc[i],
                  ifm3d::mkval<float>(xyzd_vector.data() + z_vector_offset +
                                      (ifm3d::FLOAT_DATA_SIZE * i)),
                  1e-10);
    }

  auto distVectorOffset = (3 * npts * ifm3d::FLOAT_DATA_SIZE);
  EXPECT_EQ(distance_buffer_calc.size(), npts);

  for (int i = 0; i < distance_buffer_calc.size(); ++i)
    {
      EXPECT_NEAR(distance_buffer_calc[i],
                  ifm3d::mkval<float>(xyzd_vector.data() + distVectorOffset +
                                      (ifm3d::FLOAT_DATA_SIZE * i)),
                  1e-10);
    }
}

TEST(DistanceImageInfo, DISABLED_InvalidDistImageIndex)
{
  EXPECT_EQ(ifm3d::CreateDistanceImageInfo({},
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
    readO3RByteBufferFromFile(buffer_data_file, 500, 100);
  EXPECT_NE(distance_image_info, nullptr);
  EXPECT_EQ(distance_image_info->getAmplitudeVector().size(), 0);
  EXPECT_EQ(distance_image_info->getXYZDVector().size(), 0);
}

TEST(DistanceImageInfo, AmplitudeImageConversionFromBuffer)
{
  auto distance_image_info = readO3RByteBufferFromFile(buffer_data_file);
  EXPECT_NE(distance_image_info, nullptr);
  auto npts = distance_image_info->getNPTS();
  auto amplitude_vector = distance_image_info->getAmplitudeVector();

  EXPECT_EQ(amplitude_vector.size(), npts * ifm3d::FLOAT_DATA_SIZE);
  for (int i = 0; i < Amplitude.size(); ++i)
    {
      EXPECT_NEAR(Amplitude[i],
                  ifm3d::mkval<float>(amplitude_vector.data() +
                                      i * ifm3d::FLOAT_DATA_SIZE),
                  1e-6);
    }
}

TEST(DistanceImageInfo, DistanceImageConversionFromBuffer)
{
  auto distance_image_info = readO3RByteBufferFromFile(buffer_data_file);
  EXPECT_NE(distance_image_info, nullptr);
  auto npts = distance_image_info->getNPTS();
  auto xyzd_vector = distance_image_info->getXYZDVector();

  EXPECT_EQ(xyzd_vector.size(), npts * 4 * ifm3d::FLOAT_DATA_SIZE);

  auto x_offset = compare_buffer_data_offset * ifm3d::FLOAT_DATA_SIZE;
  for (int i = 0; i < XVector.size(); ++i)
    {
      EXPECT_NEAR(XVector[i],
                  ifm3d::mkval<float>(xyzd_vector.data() + x_offset +
                                      i * ifm3d::FLOAT_DATA_SIZE),
                  1e-6);
    }

  auto y_offset = npts * ifm3d::FLOAT_DATA_SIZE +
                  compare_buffer_data_offset * ifm3d::FLOAT_DATA_SIZE;
  for (int i = 0; i < YVector.size(); ++i)
    {
      EXPECT_NEAR(YVector[i],
                  ifm3d::mkval<float>(xyzd_vector.data() + y_offset +
                                      i * ifm3d::FLOAT_DATA_SIZE),
                  1e-6);
    }

  auto z_offset = 2 * npts * ifm3d::FLOAT_DATA_SIZE +
                  compare_buffer_data_offset * ifm3d::FLOAT_DATA_SIZE;
  for (int i = 0; i < ZVector.size(); ++i)
    {
      EXPECT_NEAR(ZVector[i],
                  ifm3d::mkval<float>(xyzd_vector.data() + z_offset +
                                      i * ifm3d::FLOAT_DATA_SIZE),
                  1e-6);
    }
}
