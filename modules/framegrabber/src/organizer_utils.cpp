
#include <cstddef>
#include <cstdint>
#include <ifm3d/common/err.h>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/buffer_id.h>
#include <unordered_map>
#if defined(__GNUC__) && !defined(__clang__) && !defined(_MSC_VER)
#  include <bits/endian.h>
#elif defined(__clang__) && !defined(_MSC_VER)
#  if __has_include(<bits/endian.h>)
#    include <bits/endian.h>
#  endif
#endif
#include <algorithm>
#include <chrono>
#include <functional>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/fg/organizer_utils.h>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

constexpr auto CHUNK_OFFSET_CHUNK_SIZE = 0x0004;
constexpr auto CHUNK_OFFSET_HEADER_SIZE = 0x0008;
constexpr auto CHUNK_OFFSET_HEADER_VERSION = 0x000C;
constexpr auto CHUNK_OFFSET_IMAGE_WIDTH = 0x0010;
constexpr auto CHUNK_OFFSET_IMAGE_HEIGHT = 0x0014;
constexpr auto CHUNK_OFFSET_PIXEL_FORMAT = 0x0018;
constexpr auto CHUNK_OFFSET_TIME_STAMP = 0x001C;
constexpr auto CHUNK_OFFSET_FRAME_COUNT = 0x0020;
constexpr auto CHUNK_OFFSET_TIME_STAMP_SEC = 0x0028;
constexpr auto CHUNK_OFFSET_TIME_STAMP_NSEC = 0x002C;
constexpr auto CHUNK_OFFSET_META_DATA = 0x0030;

namespace ifm3d
{
  /**
   * @brief Describes the transport info for a logical buffer_id.
   */
  struct TransportInfo
  {
    ifm3d::buffer_id transport_id;    // The generic buffer_id it's wrapped in
                                      // (e.g., O3R_RESULT_ARRAY2D)
    std::string metadata_type_string; // The string value in the metadata that
                                      // identifies it
  };

  /**
   * @brief Static mapping of logical buffer IDs to their transport info.
   */
  const std::unordered_map<buffer_id, TransportInfo>
    LOGICAL_TO_TRANSPORT_MAPPING = {
      {ifm3d::buffer_id::O3R_ODS_RENDERED_ZONES,
       {ifm3d::buffer_id::O3R_RESULT_ARRAY2D, "ods_rendered_zones"}},
      {ifm3d::buffer_id::O3R_ODS_FLAGS,
       {ifm3d::buffer_id::O3R_RESULT_ARRAY2D, "ods_flags"}},
      {ifm3d::buffer_id::O3R_MCC_STATIC_IMAGE,
       {ifm3d::buffer_id::O3R_RESULT_ARRAY2D, "mcc_static_image"}},
      {ifm3d::buffer_id::O3R_MCC_LIVE_IMAGE,
       {ifm3d::buffer_id::O3R_RESULT_ARRAY2D, "mcc_live_image"}},
      {ifm3d::buffer_id::O3R_MCC_MOTION_IMAGE,
       {ifm3d::buffer_id::O3R_RESULT_ARRAY2D, "mcc_motion_image"}}};
}

std::size_t
ifm3d::get_format_size(ifm3d::PixelFormat fmt)
{
  switch (fmt)
    {
    case PixelFormat::FORMAT_8U:
    case PixelFormat::FORMAT_8S:
      return 1;

    case PixelFormat::FORMAT_16U:
    case PixelFormat::FORMAT_16S:
    case PixelFormat::FORMAT_16U2:
      return 2;

    case PixelFormat::FORMAT_32F3:
    case PixelFormat::FORMAT_32U:
    case PixelFormat::FORMAT_32S:
    case PixelFormat::FORMAT_32F:
      return 4;

    case PixelFormat::FORMAT_64U:
    case PixelFormat::FORMAT_64F:
      return 8;

    default:
      LOG_ERROR("Invalid pixel format => {}", static_cast<uint32_t>(fmt));
      throw ifm3d::Error(IFM3D_PIXEL_FORMAT_ERROR);
    }
}

std::size_t
ifm3d::get_format_channels(ifm3d::PixelFormat fmt)
{
  switch (fmt)
    {
    case PixelFormat::FORMAT_8U:
    case PixelFormat::FORMAT_8S:
    case PixelFormat::FORMAT_16U:
    case PixelFormat::FORMAT_16S:
    case PixelFormat::FORMAT_32U:
    case PixelFormat::FORMAT_32S:
    case PixelFormat::FORMAT_32F:
    case PixelFormat::FORMAT_64U:
    case PixelFormat::FORMAT_64F:
      return 1;

    case PixelFormat::FORMAT_16U2:
      return 2;

    case PixelFormat::FORMAT_32F3:
      return 3;

    default:
      LOG_ERROR("Invalid pixel format => {}", static_cast<uint32_t>(fmt));
      throw ifm3d::Error(IFM3D_PIXEL_FORMAT_ERROR);
    }
}

ifm3d::Buffer
ifm3d::create_1d_buffer(const std::vector<std::uint8_t>& data, std::size_t idx)
{
  std::size_t const pixeldata_offset =
    ifm3d::get_chunk_pixeldata_offset(data, idx);
  auto size = ifm3d::get_chunk_pixeldata_size(data, idx);
  auto metadata = create_metadata(data, idx);
  return create_buffer(data,
                       idx + pixeldata_offset,
                       size,
                       1,
                       PixelFormat::FORMAT_8U,
                       metadata);
}

ifm3d::Buffer
ifm3d::create_buffer(const std::vector<std::uint8_t>& data,
                     std::size_t idx,
                     std::size_t width,
                     std::size_t height)
{
  auto fmt = get_chunk_format(data, idx);
  std::size_t const pixeldata_offset = get_chunk_pixeldata_offset(data, idx);
  auto metadata = create_metadata(data, idx);
  return ifm3d::create_buffer(data,
                              idx + pixeldata_offset,
                              width,
                              height,
                              fmt,
                              metadata);
}

ifm3d::Buffer
ifm3d::create_buffer(const std::vector<std::uint8_t>& data,
                     std::size_t idx,
                     std::size_t width,
                     std::size_t height,
                     PixelFormat fmt,
                     const std::optional<json>& metadata)
{
  uint32_t const nchan = get_format_channels(fmt);
  std::size_t const fsize = get_format_size(fmt);
  ifm3d::Buffer image(width, height, nchan, fmt, metadata);

  std::size_t const npts = width * height;
  auto* ptr = image.Ptr<uint8_t>(0);

  const auto* const start = data.data() + idx;

#if !defined(_WIN32) && __BYTE_ORDER == __BIG_ENDIAN
  for (std::size_t i = 0; i < npts * nchan; ++i)
    {
      const uint8_t* src_pixel = start + i * fsize;
      uint8_t* dst_pixel = dst + i * fsize;
      std::reverse_copy(src_pixel, src_pixel + fsize, dst_pixel);
    }
#else
  std::copy(start, start + (npts * fsize), ptr);
#endif

  return image;
}

ifm3d::Buffer
ifm3d::create_xyz_buffer(const std::vector<std::uint8_t>& data,
                         std::size_t xidx,
                         std::size_t yidx,
                         std::size_t zidx,
                         std::size_t width,
                         std::size_t height,
                         ifm3d::PixelFormat fmt,
                         const std::optional<Buffer>& mask)
{
  switch (fmt)
    {
    case ifm3d::PixelFormat::FORMAT_8U:
      return create_xyz_buffer<std::uint8_t>(data,
                                             xidx,
                                             yidx,
                                             zidx,
                                             width,
                                             height,
                                             fmt,
                                             mask);
    case PixelFormat::FORMAT_8S:
      return create_xyz_buffer<std::int8_t>(data,
                                            xidx,
                                            yidx,
                                            zidx,
                                            width,
                                            height,
                                            fmt,
                                            mask);
    case PixelFormat::FORMAT_16U:
    case PixelFormat::FORMAT_16U2:
      return create_xyz_buffer<std::uint16_t>(data,
                                              xidx,
                                              yidx,
                                              zidx,
                                              width,
                                              height,
                                              fmt,
                                              mask);
    case PixelFormat::FORMAT_16S:
      return create_xyz_buffer<std::int16_t>(data,
                                             xidx,
                                             yidx,
                                             zidx,
                                             width,
                                             height,
                                             fmt,
                                             mask);
    case PixelFormat::FORMAT_32U:
      return create_xyz_buffer<std::uint32_t>(data,
                                              xidx,
                                              yidx,
                                              zidx,
                                              width,
                                              height,
                                              fmt,
                                              mask);
    case PixelFormat::FORMAT_32S:
      return create_xyz_buffer<std::int32_t>(data,
                                             xidx,
                                             yidx,
                                             zidx,
                                             width,
                                             height,
                                             fmt,
                                             mask);
    case PixelFormat::FORMAT_64U:
      return create_xyz_buffer<uint64_t>(data,
                                         xidx,
                                         yidx,
                                         zidx,
                                         width,
                                         height,
                                         fmt,
                                         mask);
    case PixelFormat::FORMAT_32F:
    case PixelFormat::FORMAT_32F3:
      return create_xyz_buffer<float>(data,
                                      xidx,
                                      yidx,
                                      zidx,
                                      width,
                                      height,
                                      fmt,
                                      mask);
    case PixelFormat::FORMAT_64F:
      return create_xyz_buffer<double>(data,
                                       xidx,
                                       yidx,
                                       zidx,
                                       width,
                                       height,
                                       fmt,
                                       mask);

    default:
      LOG_ERROR("Invalid pixel format => {}", static_cast<uint32_t>(fmt));
      throw ifm3d::Error(IFM3D_PIXEL_FORMAT_ERROR);
    }
}

std::map<ifm3d::ImageChunk, std::set<std::size_t>>
ifm3d::get_image_chunks(const std::vector<std::uint8_t>& data,
                        std::size_t start_idx,
                        std::optional<size_t> end_idx)
{
  std::map<ImageChunk, std::set<std::size_t>> chunks;

  std::size_t idx = start_idx; // start of first chunk
  std::size_t const size =
    (end_idx.has_value() ? end_idx.value() : data.size()) - 6;

  while (idx < size)
    {
      auto const chunk =
        static_cast<ImageChunk>(mkval<std::uint32_t>(data.data() + idx));
      chunks[chunk].insert(idx);

      // move to the beginning of the next chunk
      auto const incr = mkval<std::uint32_t>(data.data() + idx + 4);
      if (incr <= 0)
        {
          LOG_WARNING("Next chunk is supposedly {} bytes from the current one "
                      "... failing!",
                      incr);
          break;
        }
      idx += incr;
    }

  return chunks;
}

auto
ifm3d::find_metadata_chunk(
  const std::map<ImageChunk, std::set<std::size_t>>& chunks)
  -> decltype(chunks.end())
{
  // to get the metadata we use the confidence image for 3d and
  // the jpeg image for 2d

  auto metachunk = chunks.find(ifm3d::ImageChunk::CONFIDENCE_IMAGE);

  if (metachunk == chunks.end())
    {
      metachunk = chunks.find(ifm3d::ImageChunk::JPEG_IMAGE);
    }

  // Otherwise fall back to the first available chunk
  if (!chunks.empty())
    {
      metachunk = chunks.begin();
    }

  return metachunk;
}

std::tuple<uint32_t, uint32_t>
ifm3d::get_image_size(const std::vector<std::uint8_t>& data, std::size_t idx)
{
  if (idx + CHUNK_OFFSET_IMAGE_HEIGHT + sizeof(std::uint32_t) >= data.size())
    {
      return std::make_tuple(0, 0);
    }

  std::uint32_t const width = std::max(
    1U,
    ifm3d::mkval<std::uint32_t>(data.data() + idx + CHUNK_OFFSET_IMAGE_WIDTH));
  std::uint32_t const height =
    std::max(1U,
             ifm3d::mkval<std::uint32_t>(data.data() + idx +
                                         CHUNK_OFFSET_IMAGE_HEIGHT));

  return std::make_tuple(width, height);
}

ifm3d::PixelFormat
ifm3d::get_chunk_format(const std::vector<std::uint8_t>& data, std::size_t idx)
{
  return static_cast<PixelFormat>(
    mkval<std::uint32_t>(data.data() + idx + CHUNK_OFFSET_PIXEL_FORMAT));
}

uint32_t
ifm3d::get_chunk_frame_count(const std::vector<std::uint8_t>& data,
                             std::size_t idx)
{
  return mkval<std::uint32_t>(data.data() + idx + CHUNK_OFFSET_FRAME_COUNT);
}

std::size_t
ifm3d::get_chunk_header_version(const std::vector<std::uint8_t>& data,
                                std::size_t idx)
{
  return mkval<std::uint32_t>(data.data() + idx + CHUNK_OFFSET_HEADER_VERSION);
}

std::vector<ifm3d::TimePointT>
ifm3d::get_chunk_timestamps(const std::vector<uint8_t>& data, std::size_t idx)
{
  std::vector<TimePointT> timestamps;
  const auto header_version = ifm3d::mkval<std::uint32_t>(
    data.data() + idx + CHUNK_OFFSET_HEADER_VERSION);
  if (header_version > 1)
    {
      // Retrieve the timespamp information from the chunk
      const auto timestamp_sec = ifm3d::mkval<std::uint32_t>(
        data.data() + idx + CHUNK_OFFSET_TIME_STAMP_SEC);
      const auto timestamp_nsec = ifm3d::mkval<std::uint32_t>(
        data.data() + idx + CHUNK_OFFSET_TIME_STAMP_NSEC);
      // convert the time stamp into a TimePointT
      timestamps.emplace_back(std::chrono::seconds{timestamp_sec} +
                              std::chrono::nanoseconds{timestamp_nsec});

      // O3X device provides an offeset in Usec releative to timestamp
      // calculated as time_stamp_[0]
      const auto ethernet_timein_usec_offset = ifm3d::mkval<std::uint32_t>(
        data.data() + idx + CHUNK_OFFSET_TIME_STAMP);
      timestamps.push_back(ifm3d::TimePointT{
        timestamps[0] +
        std::chrono::microseconds{ethernet_timein_usec_offset}});
    }
  else
    {
      // There is no *big* time stamp in chunk version 1
      timestamps.push_back(std::chrono::system_clock::now());
    }

  return timestamps;
}

std::size_t
ifm3d::get_chunk_pixeldata_offset(const std::vector<std::uint8_t>& data,
                                  std::size_t idx)
{
  if (idx + CHUNK_OFFSET_HEADER_SIZE + sizeof(std::uint32_t) >= data.size())
    {
      return 0;
    }

  return ifm3d::mkval<std::uint32_t>(data.data() + idx +
                                     CHUNK_OFFSET_HEADER_SIZE);
}

std::size_t
ifm3d::get_chunk_size(const std::vector<std::uint8_t>& data, std::size_t idx)
{

  if (idx + CHUNK_OFFSET_CHUNK_SIZE + sizeof(std::uint32_t) >= data.size())
    {
      return 0;
    }

  auto chunk_size =
    ifm3d::mkval<std::uint32_t>(data.data() + idx + CHUNK_OFFSET_CHUNK_SIZE);

  return chunk_size;
}

std::size_t
ifm3d::get_chunk_pixeldata_size(const std::vector<std::uint8_t>& data,
                                std::size_t idx)
{
  if (idx + CHUNK_OFFSET_CHUNK_SIZE + sizeof(std::uint32_t) >= data.size())
    {
      return 0;
    }

  auto header_size =
    ifm3d::mkval<std::uint32_t>(data.data() + idx + CHUNK_OFFSET_HEADER_SIZE);

  auto chunk_size =
    ifm3d::mkval<std::uint32_t>(data.data() + idx + CHUNK_OFFSET_CHUNK_SIZE);

  return chunk_size - header_size;
}

void
ifm3d::mask_buffer(ifm3d::Buffer& image, const ifm3d::Buffer& mask)
{
  switch (image.DataFormat())
    {
    case ifm3d::PixelFormat::FORMAT_8U:
    case ifm3d::PixelFormat::FORMAT_8S:
      image.SetTo<std::uint8_t>(0, mask);
      break;

    case ifm3d::PixelFormat::FORMAT_16U:
    case ifm3d::PixelFormat::FORMAT_16S:
    case ifm3d::PixelFormat::FORMAT_16U2:
      image.SetTo<std::uint16_t>(0, mask);
      break;

    case ifm3d::PixelFormat::FORMAT_32U:
    case ifm3d::PixelFormat::FORMAT_32S:
    case ifm3d::PixelFormat::FORMAT_32F:
    case ifm3d::PixelFormat::FORMAT_32F3:
      image.SetTo<std::uint32_t>(0, mask);
      break;

    case ifm3d::PixelFormat::FORMAT_64U:
    case ifm3d::PixelFormat::FORMAT_64F:
      image.SetTo<std::uint64_t>(0, mask);
      break;

    default:
      break;
    }
}

bool
ifm3d::is_probably_blob(const std::vector<std::uint8_t>& data,
                        std::size_t idx,
                        std::size_t width,
                        std::size_t height)
{
  auto size = ifm3d::get_chunk_pixeldata_size(data, idx);
  auto fmt = ifm3d::get_chunk_format(data, idx);
  auto pixel_size = ifm3d::get_format_size(fmt);
  auto channel = ifm3d::get_format_channels(fmt);

  return size != (width * height * pixel_size * channel);
}

ifm3d::Buffer
ifm3d::create_pixel_mask(ifm3d::Buffer& confidence)
{
  Buffer mask =
    Buffer(confidence.Width(), confidence.Height(), 1, PixelFormat::FORMAT_8U);

  int const index = 0;
  if (confidence.DataFormat() == PixelFormat::FORMAT_16U)
    {
      std::transform(confidence.begin<std::uint16_t>(),
                     confidence.end<std::uint16_t>(),
                     mask.begin<std::uint8_t>(),
                     [](auto& value) -> uint8_t { return value & 0x1; });
    }
  else if (confidence.DataFormat() == PixelFormat::FORMAT_8U)
    {
      std::transform(confidence.begin<std::uint8_t>(),
                     confidence.end<std::uint8_t>(),
                     mask.begin<std::uint8_t>(),
                     [](auto& value) -> uint8_t { return value & 0x1; });
    }
  else
    {
      LOG_ERROR("confidence image format is not supported : ",
                static_cast<int>(confidence.DataFormat()));
      throw Error(IFM3D_CONFIDENCE_IMAGE_FORMAT_NOT_SUPPORTED);
    }

  return mask;
}

void
ifm3d::parse_data(
  const std::vector<uint8_t>& data,
  const std::set<ifm3d::buffer_id>& requested_images,
  const std::map<ifm3d::ImageChunk, std::set<std::size_t>>& chunks,
  const size_t width,
  const size_t height,
  std::map<buffer_id, BufferList>& data_blob,
  std::map<buffer_id, BufferList>& data_image)
{

  for (const auto& chunk : chunks)
    {
      if (requested_images.empty() ||
          requested_images.find(static_cast<buffer_id>(chunk.first)) !=
            requested_images.end())
        {
          for (const auto& index : chunk.second)
            {
              if (is_probably_blob(data, index, width, height))
                {
                  auto buffer = create_1d_buffer(data, index);
                  data_blob[static_cast<buffer_id>(chunk.first)].push_back(
                    buffer);
                }
              else
                {
                  auto image = create_buffer(data, index, width, height);
                  data_image[static_cast<buffer_id>(chunk.first)].push_back(
                    image);
                }
            }
        }
    }
}

void
ifm3d::mask_images(std::map<ifm3d::buffer_id, ifm3d::BufferList>& images,
                   ifm3d::Buffer& mask,
                   const std::function<bool(ifm3d::buffer_id id)>& should_mask)
{
  for (auto& [buffer_id_value, buffers] : images)
    {
      if (should_mask(buffer_id_value))
        {
          for (auto& buffer : buffers)
            {
              mask_buffer(buffer, mask);
            }
        }
    }
}

bool
ifm3d::has_metadata(const std::vector<std::uint8_t>& data, std::size_t idx)
{
  auto version = get_chunk_header_version(data, idx);
  return version >= 3;
}

ifm3d::json
ifm3d::create_metadata(const std::vector<std::uint8_t>& data, std::size_t idx)
{
  if (has_metadata(data, idx))
    {
      std::string const metadata(std::string(
        reinterpret_cast<const char*>(data.data() + idx +
                                      CHUNK_OFFSET_META_DATA),
        get_chunk_pixeldata_offset(data, idx) - CHUNK_OFFSET_META_DATA));
      return ifm3d::json::parse(metadata);
    }
  return {};
}

// Helper function to map metadata 'type' string to its buffer_id enum
ifm3d::buffer_id
ifm3d::map_metadata_to_buffer_id(const ifm3d::Buffer& buffer)
{
  const auto& metadata = buffer.Metadata();

  const auto& result_node = metadata.at("result");
  const auto& type_node = result_node.at("type");
  if (!type_node.is_string())
    {
      throw ifm3d::Error(
        IFM3D_JSON_ERROR,
        "The 'type' field in buffer metadata is not a string.");
    }
  const std::string type_str = type_node.get<std::string>();
  // Look up the string in the LOGICAL_TO_TRANSPORT_MAPPING
  for (const auto& pair : ifm3d::LOGICAL_TO_TRANSPORT_MAPPING)
    {
      if (pair.second.metadata_type_string == type_str)
        {
          return pair.first; // Return the logical buffer_id
        }
    }

  // If the type string is not found in our known mappings
  LOG_ERROR("Unknown buffer type string in metadata: '{}'", type_str);
  throw ifm3d::Error(IFM3D_BUFFER_ID_NOT_AVAILABLE);
}

std::map<ifm3d::buffer_id, ifm3d::BufferList>
ifm3d::map_logical_buffers(
  const std::map<ifm3d::buffer_id, ifm3d::BufferList>& images,
  const std::set<ifm3d::buffer_id>& requested_images)
{
  std::map<ifm3d::buffer_id, ifm3d::BufferList> mapped_images;

  for (const auto& requested_id : requested_images)
    {
      auto mapping_it = ifm3d::LOGICAL_TO_TRANSPORT_MAPPING.find(requested_id);

      // --- Case 1: Direct (non-transport) buffer ---
      if (mapping_it == ifm3d::LOGICAL_TO_TRANSPORT_MAPPING.end())
        {
          auto img_it = images.find(requested_id);
          if (img_it != images.end())
            {
              mapped_images[requested_id] = img_it->second;
            }
          continue;
        }

      // --- Case 2: Logical → Transport mapped buffer ---
      const auto& transport_id = mapping_it->second.transport_id;
      auto img_it = images.find(transport_id);
      if (img_it == images.end())
        {
          continue;
        }

      const auto& transport_buffers = img_it->second;
      mapped_images[requested_id].reserve(transport_buffers.size());

      for (const auto& buf : transport_buffers)
        {
          try
            {
              if (ifm3d::map_metadata_to_buffer_id(buf) == requested_id)
                {
                  mapped_images[requested_id].push_back(buf);
                }
            }
          catch (const ifm3d::Error& e)
            {
              LOG_ERROR("Mapping error for buffer_id {} : {}",
                        static_cast<int>(transport_id),
                        e.what());
              throw ifm3d::Error(IFM3D_BUFFER_ID_NOT_AVAILABLE);
            }
        }
    }
  return mapped_images;
}