/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "o3r_organizer3D.hpp"
#include <cstddef>
#include <cstdint>
#include <functional>
#include <ifm3d/common/err.h>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/buffer_id.h>
#include <ifm3d/fg/distance_image_info.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/fg/organizer.h>
#include <ifm3d/fg/organizer_utils.h>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

ifm3d::Organizer::Result
ifm3d::O3ROrganizer3D::Organize(const std::vector<uint8_t>& data,
                                const std::set<buffer_id>& requested_images,
                                const bool masking)
{
  std::map<buffer_id, BufferList> images;

  auto chunks = get_image_chunks(data, IMG_BUFF_START);

  auto metachunk = find_metadata_chunk(chunks);

  // if we do not have a meta chunk we cannot go further
  if (metachunk == chunks.end())
    {
      throw Error(IFM3D_IMG_CHUNK_NOT_FOUND, "No meta chunk found!");
    }

  // get the image dimensions
  auto [width, height] = get_image_size(data, *(metachunk->second.begin()));

  auto timestamps = get_chunk_timestamps(data, *(metachunk->second.begin()));
  auto frame_count = get_chunk_frame_count(data, *(metachunk->second.begin()));

  // for an O3R device, a distance_image_info object will be created
  // for others a nullptr is returned
  std::shared_ptr<DistanceImageInfo> distance_image_info;
  if (chunks.find(ImageChunk::TOF_INFO) != chunks.end() &&
      chunks.find(ImageChunk::RADIAL_DISTANCE_IMAGE) != chunks.end() &&
      chunks.find(ImageChunk::NORM_AMPLITUDE_IMAGE) != chunks.end())
    {
      distance_image_info = create_distance_image_info(
        data,
        *(chunks.at(ImageChunk::TOF_INFO).begin()),
        *(chunks.at(ImageChunk::RADIAL_DISTANCE_IMAGE).begin()),
        *(chunks.at(ImageChunk::NORM_AMPLITUDE_IMAGE).begin()),
        width,
        height);

      chunks.erase(ImageChunk::NORM_AMPLITUDE_IMAGE);
      chunks.erase(ImageChunk::RADIAL_DISTANCE_IMAGE);
    }

  std::map<buffer_id, BufferList> data_blob;
  std::map<buffer_id, BufferList> data_image;
  ifm3d::parse_data(data,
                    requested_images,
                    chunks,
                    width,
                    height,
                    data_blob,
                    data_image);

  images.insert(data_image.begin(), data_image.end());
  images.insert(data_blob.begin(), data_blob.end());

  std::optional<ifm3d::Buffer> mask;
  if (masking)
    {
      if (images.find(buffer_id::CONFIDENCE_IMAGE) != images.end())
        {
          mask =
            create_pixel_mask(images[ifm3d::buffer_id::CONFIDENCE_IMAGE][0]);
          mask_images(data_image, mask.value(), [this](auto&& p_h1) {
            return should_mask(std::forward<decltype(p_h1)>(p_h1));
          });
        }
    }

  if (distance_image_info != nullptr)
    {
      auto extracted = extract_distance_image_info(distance_image_info, mask);
      for (auto& itr : extracted)
        {
          images.insert({itr.first, {itr.second}});
        }

      if (images.find(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE) != images.end())
        {
          auto dist_noise_buffer =
            distance_image_info->ApplyDistanceResolution(
              images[static_cast<buffer_id>(
                ifm3d::ImageChunk::RADIAL_DISTANCE_NOISE)][0]);

          images[static_cast<buffer_id>(
            ifm3d::buffer_id::RADIAL_DISTANCE_NOISE)] = {dist_noise_buffer};
        }
    }
  else if (requested_images.empty() ||
           requested_images.find(static_cast<buffer_id>(buffer_id::XYZ)) !=
             requested_images.end())
    {
      auto x = chunks.find(ImageChunk::CARTESIAN_X_COMPONENT);
      auto y = chunks.find(ImageChunk::CARTESIAN_Y_COMPONENT);
      auto z = chunks.find(ImageChunk::CARTESIAN_Z_COMPONENT);

      if (x != chunks.end() && y != chunks.end() && z != chunks.end())
        {
          auto fmt = get_chunk_format(data, *(x->second.begin()));
          auto xyz = create_xyz_buffer(data,
                                       *(x->second.begin()),
                                       *(y->second.begin()),
                                       *(z->second.begin()),
                                       width,
                                       height,
                                       fmt,
                                       mask);

          images[static_cast<buffer_id>(buffer_id::XYZ)] = {xyz};
        }
    }

  return {images, timestamps, frame_count};
}

std::map<ifm3d::buffer_id, ifm3d::Buffer>
ifm3d::O3ROrganizer3D::extract_distance_image_info(
  const std::shared_ptr<DistanceImageInfo>& distance_image_info,
  const std::optional<Buffer>& mask)
{
  auto width = distance_image_info->GetWidth();
  auto height = distance_image_info->GetHeight();
  auto npts = distance_image_info->GetNpts();

  std::vector<std::uint8_t> const xyzd_bytes =
    distance_image_info->GetXyzdVector();

  std::vector<std::uint8_t> const ampl_bytes =
    distance_image_info->GetAmplitudeVector();

  auto distance =
    create_buffer(xyzd_bytes,
                  static_cast<std::size_t>(npts) * 3 * sizeof(float),
                  width,
                  height,
                  PixelFormat::FORMAT_32F);

  auto amplitude =
    create_buffer(ampl_bytes, 0, width, height, PixelFormat::FORMAT_32F);

  if (mask.has_value())
    {
      mask_buffer(distance, mask.value());
      mask_buffer(amplitude, mask.value());
    }

  auto xyz =
    create_xyz_buffer(xyzd_bytes,
                      0,
                      npts * sizeof(float),
                      static_cast<std::size_t>(npts) * 2 * sizeof(float),
                      width,
                      height,
                      PixelFormat::FORMAT_32F,
                      mask);
  auto extrinsic_param = create_buffer_from_vector<float>(
    distance_image_info->GetExtrinsicOpticToUser());

  auto intrinsic_param = create_buffer_from_struct<IntrinsicCalibration>(
    distance_image_info->GetIntrinsicCalibration());

  auto inv_intrinsic_param = create_buffer_from_struct<IntrinsicCalibration>(
    distance_image_info->GetInverseIntrinsicCalibration());

  return {
    {static_cast<buffer_id>(ImageChunk::NORM_AMPLITUDE_IMAGE), amplitude},
    {static_cast<buffer_id>(ImageChunk::RADIAL_DISTANCE_IMAGE), distance},
    {static_cast<buffer_id>(buffer_id::XYZ), xyz},
    {static_cast<buffer_id>(buffer_id::EXTRINSIC_CALIB), extrinsic_param},
    {static_cast<buffer_id>(buffer_id::INTRINSIC_CALIB), intrinsic_param},
    {static_cast<buffer_id>(buffer_id::INVERSE_INTRINSIC_CALIBRATION),
     inv_intrinsic_param},
  };
}

bool
ifm3d::O3ROrganizer3D::should_mask(buffer_id id)
{
  switch (id)
    {
    case static_cast<buffer_id>(ImageChunk::UNIT_VECTOR_ALL):
    case static_cast<buffer_id>(ImageChunk::CONFIDENCE_IMAGE):
    case static_cast<buffer_id>(ImageChunk::JPEG_IMAGE):
    case static_cast<buffer_id>(ImageChunk::TOF_INFO):
      return false;

    default:
      return true;
    }
}
