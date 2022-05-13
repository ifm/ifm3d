/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <default_organizer.hpp>
#include <glog/logging.h>
#include <ifm3d/camera/err.h>
#include <ifm3d/fg/image.h>
#include <ifm3d/fg/organizer_utils.h>
#include <ifm3d/fg/distance_image_info.h>

ifm3d::Image
ifm3d::DefaultOrganizer::CreatePixelMask(Image& confidence)
{
  Image mask =
    Image(confidence.width(), confidence.height(), 1, pixel_format::FORMAT_8U);

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
      throw error_t(IFM3D_CONFIDENCE_IMAGE_FORMAT_NOT_SUPPORTED);
    }

  return mask;
}

ifm3d::Organizer::Result
ifm3d::DefaultOrganizer::Organize(const std::vector<uint8_t>& data,
                                  const std::set<image_id>& requested_images)
{
  std::map<image_id, Image> images;

  auto chunks = get_image_chunks(data, IMG_BUFF_START);

  auto metachunk = find_metadata_chunk(chunks);

  // if we do not have a meta chunk we cannot go further
  if (metachunk == chunks.end())
    {
      LOG(ERROR) << "No meta chunk found!";
      throw error_t(IFM3D_IMG_CHUNK_NOT_FOUND);
    }

  // get the image dimensions
  auto [width, height] = get_image_size(data, metachunk->second);
  std::uint32_t npts = width * height;

  auto timestamps = get_chunk_timestamps(data, metachunk->second);

  // for an O3R device, a distance_image_info object will be created
  // for others a nullptr is returned
  std::shared_ptr<DistanceImageInfo> distance_image_info;
  if (chunks.find(image_chunk::O3R_DISTANCE_IMAGE_INFORMATION) !=
        chunks.end() &&
      chunks.find(image_chunk::RADIAL_DISTANCE) != chunks.end() &&
      chunks.find(image_chunk::AMPLITUDE) != chunks.end())
    {
      distance_image_info = CreateDistanceImageInfo(
        data,
        chunks.at(image_chunk::O3R_DISTANCE_IMAGE_INFORMATION),
        chunks.at(image_chunk::RADIAL_DISTANCE),
        chunks.at(image_chunk::AMPLITUDE),
        width,
        height);

      chunks.erase(image_chunk::AMPLITUDE);
      chunks.erase(image_chunk::RADIAL_DISTANCE);
      chunks.erase(image_chunk::O3R_DISTANCE_IMAGE_INFORMATION);
    }

  std::optional<Image> mask;

  if (chunks.find(image_chunk::CONFIDENCE) != chunks.end())
    {
      auto confidence =
        create_image(data, chunks[image_chunk::CONFIDENCE], width, height);

      mask = CreatePixelMask(confidence);

      chunks.erase(image_chunk::CONFIDENCE);
    }

  // JPEG chunk has format set as 32U, but is actually 8U, so we need to
  // manually extract here
  if (chunks.find(image_chunk::JPEG) != chunks.end())
    {
      const auto idx = chunks[image_chunk::JPEG];

      std::size_t pixeldata_offset = get_chunk_pixeldata_offset(data, idx);
      auto size = ifm3d::get_chunk_pixeldata_size(data, idx);

      auto jpeg = create_image(data,
                               idx + pixeldata_offset,
                               size,
                               1,
                               pixel_format::FORMAT_8U);

      images[static_cast<image_id>(image_chunk::JPEG)] = jpeg;

      chunks.erase(image_chunk::JPEG);
    }

  for (const auto& chunk : chunks)
    {
      if (requested_images.empty() ||
          requested_images.find(static_cast<image_id>(chunk.first)) !=
            requested_images.end())
        {
          auto image = create_image(data, chunk.second, width, height);

          if (mask.has_value() &&
              ShouldMask(static_cast<image_id>(chunk.first)))
            {
              mask_image(image, mask.value());
            }

          images[static_cast<image_id>(chunk.first)] = image;
        }
    }

  if (requested_images.empty() || requested_images.find(static_cast<image_id>(
                                    image_id::XYZ)) != requested_images.end())
    {
      if (distance_image_info != nullptr)
        {
          auto extracted = ExtractDistanceImageInfo(distance_image_info, mask);
          images.insert(extracted.begin(), extracted.end());
        }
      else
        {
          auto x = chunks.find(image_chunk::CARTESIAN_X);
          auto y = chunks.find(image_chunk::CARTESIAN_Y);
          auto z = chunks.find(image_chunk::CARTESIAN_Z);

          if (x != chunks.end() && y != chunks.end() && z != chunks.end())
            {
              auto fmt = get_chunk_format(data, x->second);
              auto xyz = create_xyz_image(data,
                                          x->second,
                                          y->second,
                                          z->second,
                                          width,
                                          height,
                                          fmt,
                                          mask);

              images[static_cast<image_id>(image_id::XYZ)] = xyz;
            }
        }
    }

  return {images, timestamps};
}

std::map<ifm3d::image_id, ifm3d::Image>
ifm3d::DefaultOrganizer::ExtractDistanceImageInfo(
  std::shared_ptr<DistanceImageInfo> distance_image_info,
  const std::optional<Image>& mask)
{
  auto width = distance_image_info->getWidth();
  auto height = distance_image_info->getHeight();
  auto npts = distance_image_info->getNPTS();

  std::vector<std::uint8_t> xyzd_bytes = distance_image_info->getXYZDVector();

  std::vector<std::uint8_t> ampl_bytes =
    distance_image_info->getAmplitudeVector();

  auto distance = create_image(xyzd_bytes,
                               npts * 3 * sizeof(float),
                               width,
                               height,
                               pixel_format::FORMAT_32F);

  auto amplitude =
    create_image(ampl_bytes, 0, width, height, pixel_format::FORMAT_32F);

  if (mask.has_value())
    {
      mask_image(distance, mask.value());
      mask_image(amplitude, mask.value());
    }

  auto xyz = create_xyz_image(xyzd_bytes,
                              0,
                              npts * sizeof(float),
                              npts * 2 * sizeof(float),
                              width,
                              height,
                              pixel_format::FORMAT_32F,
                              mask);

  return {
    {static_cast<image_id>(image_chunk::AMPLITUDE), amplitude},
    {static_cast<image_id>(image_chunk::RADIAL_DISTANCE), distance},
    {static_cast<image_id>(image_id::XYZ), xyz},
  };
}

bool
ifm3d::DefaultOrganizer::ShouldMask(image_id id)
{
  switch (id)
    {
    case static_cast<image_id>(image_chunk::UNIT_VECTOR_ALL):
    case static_cast<image_id>(image_chunk::CONFIDENCE):
    case static_cast<image_id>(image_chunk::JPEG):
      return false;

    default:
      return true;
    }
}