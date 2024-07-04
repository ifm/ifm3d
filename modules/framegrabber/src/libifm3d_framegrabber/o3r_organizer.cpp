/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <o3r_organizer.hpp>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/organizer_utils.h>
#include <ifm3d/fg/distance_image_info.h>

ifm3d::Organizer::Result
ifm3d::O3ROrganizer::Organize(const std::vector<uint8_t>& data,
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
  std::uint32_t npts = width * height;

  auto timestamps = get_chunk_timestamps(data, *(metachunk->second.begin()));
  auto frame_count = get_chunk_frame_count(data, *(metachunk->second.begin()));

  std::map<buffer_id, BufferList> data_blob, data_image;
  ifm3d::parse_data(data,
                    requested_images,
                    chunks,
                    width,
                    height,
                    data_blob,
                    data_image);

  images.insert(data_image.begin(), data_image.end());
  images.insert(data_blob.begin(), data_blob.end());

  return {images, timestamps, frame_count};
}
