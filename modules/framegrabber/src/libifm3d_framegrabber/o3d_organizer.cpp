/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include <functional>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/organizer_utils.h>
#include <ifm3d/fg/distance_image_info.h>
#include <o3d_organizer.hpp>

namespace ifm3d
{
  namespace
  {
    // exposure time
    constexpr size_t SIZE_OF_O3D_EXPOSURE_TIME_KEY_STRING = 5;
    constexpr size_t SIZE_OF_O3D_EXPOSURE_TIME_VALUES = 3 * 4;
    constexpr size_t SiZE_OF_O3D_EXPOSURE_TIME_DATA =
      SIZE_OF_O3D_EXPOSURE_TIME_KEY_STRING + SIZE_OF_O3D_EXPOSURE_TIME_VALUES;

    // illumination temperature
    constexpr size_t SIZE_OF_O3D_ILLUMINATION_TEMPRATURE_KEY_STRING = 9;
    constexpr size_t SIZE_OF_O3D_ILLUMINATION_TEMPERATURE_VALUE = 4;
    constexpr size_t SiZE_OF_O3D_ILLUNIMATION_TEMPERATURE_DATA =
      SIZE_OF_O3D_ILLUMINATION_TEMPRATURE_KEY_STRING +
      SIZE_OF_O3D_ILLUMINATION_TEMPERATURE_VALUE;
  };
};

ifm3d::Organizer::Result
ifm3d::O3DOrganizer::Organize(const std::vector<uint8_t>& data,
                              const std::set<buffer_id>& requested_images,
                              const bool masking)
{
  ifm3d::BufferDataListMap images;
  size_t end_idx = data.size() - 6;
  // iilumination temperature data if present will always come last as it last
  // data in schema Note : this is automatically done with sorting provided by
  // std::set.
  if (requested_images.count(ifm3d::buffer_id::ILLUMINATION_TEMP) == 1)
    {

      size_t illumination_temp_idx =
        end_idx - ifm3d::SiZE_OF_O3D_ILLUNIMATION_TEMPERATURE_DATA;
      ifm3d::Buffer illumination_temp_buffer = ifm3d::create_buffer(
        data,
        illumination_temp_idx +
          ifm3d::SIZE_OF_O3D_ILLUMINATION_TEMPRATURE_KEY_STRING,
        ifm3d::SIZE_OF_O3D_ILLUMINATION_TEMPERATURE_VALUE,
        1,
        ifm3d::pixel_format::FORMAT_8U);
      images[ifm3d::buffer_id::ILLUMINATION_TEMP] = {illumination_temp_buffer};

      end_idx = illumination_temp_idx;
    }

  // Exposure time data if present will always come 2nd last as it
  if (requested_images.count(ifm3d::buffer_id::EXPOSURE_TIME) == 1)
    {

      size_t exposure_time_idx =
        end_idx - ifm3d::SiZE_OF_O3D_EXPOSURE_TIME_DATA;
      ifm3d::Buffer exposure_time_buffer = ifm3d::create_buffer(
        data,
        exposure_time_idx + ifm3d::SIZE_OF_O3D_EXPOSURE_TIME_KEY_STRING,
        ifm3d::SIZE_OF_O3D_EXPOSURE_TIME_VALUES,
        1,
        ifm3d::pixel_format::FORMAT_8U);
      images[ifm3d::buffer_id::EXPOSURE_TIME] = {exposure_time_buffer};

      end_idx = exposure_time_idx;
    }

  auto chunks = get_image_chunks(data, IMG_BUFF_START, end_idx);

  auto metachunk = find_metadata_chunk(chunks);

  // if we do not have a meta chunk we cannot go further
  if (metachunk == chunks.end())
    {
      LOG_ERROR("No meta chunk found!");
      throw Error(IFM3D_IMG_CHUNK_NOT_FOUND);
    }

  // get the image dimensions
  auto [width, height] = get_image_size(data, *(metachunk->second.begin()));
  std::uint32_t npts = width * height;

  auto timestamps = get_chunk_timestamps(data, *(metachunk->second.begin()));
  auto frame_count = get_chunk_frame_count(data, *(metachunk->second.begin()));

  if (chunks.find(image_chunk::CARTESIAN_ALL) != chunks.end())
    {
      size_t cart_all_idx = *(chunks[image_chunk::CARTESIAN_ALL].begin());
      size_t x_idx =
        cart_all_idx + get_chunk_pixeldata_offset(data, cart_all_idx);

      size_t y_idx = x_idx + get_chunk_size(data, x_idx);
      size_t z_idx = y_idx + get_chunk_size(data, y_idx);

      chunks[image_chunk::CARTESIAN_X_COMPONENT] = {x_idx};
      chunks[image_chunk::CARTESIAN_Y_COMPONENT] = {y_idx};
      chunks[image_chunk::CARTESIAN_Z_COMPONENT] = {z_idx};
      chunks.erase(image_chunk::CARTESIAN_ALL);
    }

  std::map<buffer_id, BufferList> data_blob, data_image;
  ifm3d::parse_data(data,
                    requested_images,
                    chunks,
                    width,
                    height,
                    data_blob,
                    data_image);

  std::optional<ifm3d::Buffer> mask;

  if (masking)
    {
      if (images.find(buffer_id::CONFIDENCE_IMAGE) != images.end())
        {
          mask =
            create_pixel_mask(images[ifm3d::buffer_id::CONFIDENCE_IMAGE][0]);
          mask_images(data_image,
                      mask.value(),
                      std::bind(&ifm3d::O3DOrganizer::ShouldMask,
                                this,
                                std::placeholders::_1));
        }
    }

  images.insert(data_image.begin(), data_image.end());
  images.insert(data_blob.begin(), data_blob.end());

  // special case of XYZ buffer
  if (requested_images.empty() || requested_images.find(static_cast<buffer_id>(
                                    buffer_id::XYZ)) != requested_images.end())
    {
      auto x = chunks.find(image_chunk::CARTESIAN_X_COMPONENT);
      auto y = chunks.find(image_chunk::CARTESIAN_Y_COMPONENT);
      auto z = chunks.find(image_chunk::CARTESIAN_Z_COMPONENT);

      if (x != chunks.end() && y != chunks.end() && z != chunks.end())
        {
          auto fmt = get_chunk_format(data, *(x->second.begin()));
          auto xyz = create_xyz_buffer(
            data,
            *(x->second.begin()) +
              get_chunk_pixeldata_offset(data, *(x->second.begin())),
            *(y->second.begin()) +
              get_chunk_pixeldata_offset(data, *(y->second.begin())),
            *(z->second.begin()) +
              get_chunk_pixeldata_offset(data, *(z->second.begin())),
            width,
            height,
            fmt,
            mask);
          images[static_cast<buffer_id>(buffer_id::XYZ)] = {xyz};
        }
    }
  return {images, timestamps, frame_count};
}

bool
ifm3d::O3DOrganizer::ShouldMask(buffer_id id)
{
  switch (id)
    {
    case static_cast<buffer_id>(image_chunk::UNIT_VECTOR_ALL):
    case static_cast<buffer_id>(image_chunk::CONFIDENCE_IMAGE):
      return false;

    default:
      return true;
    }
}