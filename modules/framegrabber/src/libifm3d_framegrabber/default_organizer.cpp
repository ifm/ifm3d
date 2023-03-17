/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <default_organizer.hpp>
#include <glog/logging.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/organizer_utils.h>
#include <ifm3d/fg/distance_image_info.h>

ifm3d::Buffer
ifm3d::DefaultOrganizer::CreatePixelMask(Buffer& confidence)
{
  Buffer mask = Buffer(confidence.width(),
                       confidence.height(),
                       1,
                       pixel_format::FORMAT_8U);

  int index = 0;
  if (confidence.dataFormat() == pixel_format::FORMAT_16U)
    {
      std::transform(confidence.begin<std::uint16_t>(),
                     confidence.end<std::uint16_t>(),
                     mask.begin<std::uint8_t>(),
                     [](auto& value) -> uint8_t { return value & 0x1; });
    }
  else if (confidence.dataFormat() == pixel_format::FORMAT_8U)
    {
      std::transform(confidence.begin<std::uint8_t>(),
                     confidence.end<std::uint8_t>(),
                     mask.begin<std::uint8_t>(),
                     [](auto& value) -> uint8_t { return value & 0x1; });
    }
  else
    {
      LOG(ERROR) << "confidence image format is not supported : "
                 << (int)confidence.dataFormat();
      throw Error(IFM3D_CONFIDENCE_IMAGE_FORMAT_NOT_SUPPORTED);
    }

  return mask;
}

ifm3d::Organizer::Result
ifm3d::DefaultOrganizer::Organize(const std::vector<uint8_t>& data,
                                  const std::set<buffer_id>& requested_images,
                                  const bool masking)
{
  std::map<buffer_id, Buffer> images;

  auto chunks = get_image_chunks(data, IMG_BUFF_START);

  auto metachunk = find_metadata_chunk(chunks);

  // if we do not have a meta chunk we cannot go further
  if (metachunk == chunks.end())
    {
      LOG(ERROR) << "No meta chunk found!";
      throw Error(IFM3D_IMG_CHUNK_NOT_FOUND);
    }

  // get the image dimensions
  auto [width, height] = get_image_size(data, metachunk->second);
  std::uint32_t npts = width * height;

  auto timestamps = get_chunk_timestamps(data, metachunk->second);
  auto frame_count = get_chunk_frame_count(data, metachunk->second);

  // for an O3R device, a distance_image_info object will be created
  // for others a nullptr is returned
  std::shared_ptr<DistanceImageInfo> distance_image_info;
  if (chunks.find(image_chunk::TOF_INFO) != chunks.end() &&
      chunks.find(image_chunk::RADIAL_DISTANCE_IMAGE) != chunks.end() &&
      chunks.find(image_chunk::NORM_AMPLITUDE_IMAGE) != chunks.end())
    {
      distance_image_info =
        CreateDistanceImageInfo(data,
                                chunks.at(image_chunk::TOF_INFO),
                                chunks.at(image_chunk::RADIAL_DISTANCE_IMAGE),
                                chunks.at(image_chunk::NORM_AMPLITUDE_IMAGE),
                                width,
                                height);

      chunks.erase(image_chunk::NORM_AMPLITUDE_IMAGE);
      chunks.erase(image_chunk::RADIAL_DISTANCE_IMAGE);
    }

  std::optional<Buffer> mask;

  if (chunks.find(image_chunk::CONFIDENCE_IMAGE) != chunks.end())
    {
      auto confidence = create_buffer(data,
                                      chunks[image_chunk::CONFIDENCE_IMAGE],
                                      width,
                                      height);
      if (masking)
        {
          mask = CreatePixelMask(confidence);
        }

      images[ifm3d::buffer_id::CONFIDENCE_IMAGE] = confidence;
      chunks.erase(image_chunk::CONFIDENCE_IMAGE);
    }

  for (const auto& chunk : chunks)
    {
      // for O3D only as this chunk donot need to create image
      if (chunk.first == ifm3d::image_chunk::DIAGNOSTIC ||
          chunk.first == ifm3d::image_chunk::EXTRINSIC_CALIB)
        {
          continue;
        }
      if (requested_images.empty() ||
          requested_images.find(static_cast<buffer_id>(chunk.first)) !=
            requested_images.end())
        {
          if (is_probably_blob(data, chunk.second, width, height))
            {
              auto buffer = create_1d_buffer(data, chunk.second);
              images[static_cast<buffer_id>(chunk.first)] = buffer;
            }
          else
            {
              auto image = create_buffer(data, chunk.second, width, height);
              if (mask.has_value() &&
                  ShouldMask(static_cast<buffer_id>(chunk.first)))
                {
                  mask_buffer(image, mask.value());
                }
              images[static_cast<buffer_id>(chunk.first)] = image;
            }
        }
    }
  if (distance_image_info != nullptr)
    {
      auto extracted = ExtractDistanceImageInfo(distance_image_info, mask);
      images.insert(extracted.begin(), extracted.end());

      if (images.find(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE) != images.end())
        {
          auto dist_noise_buffer =
            distance_image_info->applyDistanceResolution(
              images[static_cast<buffer_id>(
                ifm3d::image_chunk::RADIAL_DISTANCE_NOISE)]);

          images[static_cast<buffer_id>(
            ifm3d::buffer_id::RADIAL_DISTANCE_NOISE)] = dist_noise_buffer;
        }
    }
  else if (requested_images.empty() ||
           requested_images.find(static_cast<buffer_id>(buffer_id::XYZ)) !=
             requested_images.end())
    {
      auto x = chunks.find(image_chunk::CARTESIAN_X_COMPONENT);
      auto y = chunks.find(image_chunk::CARTESIAN_Y_COMPONENT);
      auto z = chunks.find(image_chunk::CARTESIAN_Z_COMPONENT);

      if (x != chunks.end() && y != chunks.end() && z != chunks.end())
        {
          auto fmt = get_chunk_format(data, x->second);
          auto xyz = create_xyz_buffer(data,
                                       x->second,
                                       y->second,
                                       z->second,
                                       width,
                                       height,
                                       fmt,
                                       mask);

          images[static_cast<buffer_id>(buffer_id::XYZ)] = xyz;
        }
    }

  return {images, timestamps, frame_count};
}

std::map<ifm3d::buffer_id, ifm3d::Buffer>
ifm3d::DefaultOrganizer::ExtractDistanceImageInfo(
  std::shared_ptr<DistanceImageInfo> distance_image_info,
  const std::optional<Buffer>& mask)
{
  auto width = distance_image_info->getWidth();
  auto height = distance_image_info->getHeight();
  auto npts = distance_image_info->getNPTS();

  std::vector<std::uint8_t> xyzd_bytes = distance_image_info->getXYZDVector();

  std::vector<std::uint8_t> ampl_bytes =
    distance_image_info->getAmplitudeVector();

  auto distance = create_buffer(xyzd_bytes,
                                npts * 3 * sizeof(float),
                                width,
                                height,
                                pixel_format::FORMAT_32F);

  auto amplitude =
    create_buffer(ampl_bytes, 0, width, height, pixel_format::FORMAT_32F);

  if (mask.has_value())
    {
      mask_buffer(distance, mask.value());
      mask_buffer(amplitude, mask.value());
    }

  auto xyz = create_xyz_buffer(xyzd_bytes,
                               0,
                               npts * sizeof(float),
                               npts * 2 * sizeof(float),
                               width,
                               height,
                               pixel_format::FORMAT_32F,
                               mask);
  auto extrinsic_param = create_buffer_from_vector<float>(
    distance_image_info->getExtrinsicOpticToUser());

  auto intrinsic_param = create_buffer_from_struct<IntrinsicCalibration>(
    distance_image_info->getIntrinsicCalibration());

  auto inv_intrinsic_param = create_buffer_from_struct<IntrinsicCalibration>(
    distance_image_info->getInverseIntrinsicCalibration());

  return {
    {static_cast<buffer_id>(image_chunk::NORM_AMPLITUDE_IMAGE), amplitude},
    {static_cast<buffer_id>(image_chunk::RADIAL_DISTANCE_IMAGE), distance},
    {static_cast<buffer_id>(buffer_id::XYZ), xyz},
    {static_cast<buffer_id>(buffer_id::EXTRINSIC_CALIB), extrinsic_param},
    {static_cast<buffer_id>(buffer_id::INTRINSIC_CALIB), intrinsic_param},
    {static_cast<buffer_id>(buffer_id::INVERSE_INTRINSIC_CALIBRATION),
     inv_intrinsic_param},
  };
}

bool
ifm3d::DefaultOrganizer::ShouldMask(buffer_id id)
{
  switch (id)
    {
    case static_cast<buffer_id>(image_chunk::UNIT_VECTOR_ALL):
    case static_cast<buffer_id>(image_chunk::CONFIDENCE_IMAGE):
    case static_cast<buffer_id>(image_chunk::JPEG_IMAGE):
    case static_cast<buffer_id>(image_chunk::TOF_INFO):
      return false;

    default:
      return true;
    }
}
