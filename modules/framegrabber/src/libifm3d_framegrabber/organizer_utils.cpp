
#include <cstddef>
#include <cstdint>
#include <ifm3d/common/err.h>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/buffer_id.h>
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

std::size_t
ifm3d::get_format_size(ifm3d::pixel_format fmt)
{
  switch (fmt)
    {
    case pixel_format::FORMAT_8U:
    case pixel_format::FORMAT_8S:
      return 1;

    case pixel_format::FORMAT_16U:
    case pixel_format::FORMAT_16S:
    case pixel_format::FORMAT_16U2:
      return 2;

    case pixel_format::FORMAT_32F3:
    case pixel_format::FORMAT_32U:
    case pixel_format::FORMAT_32S:
    case pixel_format::FORMAT_32F:
      return 4;

    case pixel_format::FORMAT_64U:
    case pixel_format::FORMAT_64F:
      return 8;

    default:
      LOG_ERROR("Invalid pixel format => {}", static_cast<uint32_t>(fmt));
      throw ifm3d::Error(IFM3D_PIXEL_FORMAT_ERROR);
    }
}

std::size_t
ifm3d::get_format_channels(ifm3d::pixel_format fmt)
{
  switch (fmt)
    {
    case pixel_format::FORMAT_8U:
    case pixel_format::FORMAT_8S:
    case pixel_format::FORMAT_16U:
    case pixel_format::FORMAT_16S:
    case pixel_format::FORMAT_32U:
    case pixel_format::FORMAT_32S:
    case pixel_format::FORMAT_32F:
    case pixel_format::FORMAT_64U:
    case pixel_format::FORMAT_64F:
      return 1;

    case pixel_format::FORMAT_16U2:
      return 2;

    case pixel_format::FORMAT_32F3:
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
                       pixel_format::FORMAT_8U,
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
                     pixel_format fmt,
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
                         ifm3d::pixel_format fmt,
                         const std::optional<Buffer>& mask)
{
  switch (fmt)
    {
    case ifm3d::pixel_format::FORMAT_8U:
      return create_xyz_buffer<std::uint8_t>(data,
                                             xidx,
                                             yidx,
                                             zidx,
                                             width,
                                             height,
                                             fmt,
                                             mask);
    case pixel_format::FORMAT_8S:
      return create_xyz_buffer<std::int8_t>(data,
                                            xidx,
                                            yidx,
                                            zidx,
                                            width,
                                            height,
                                            fmt,
                                            mask);
    case pixel_format::FORMAT_16U:
    case pixel_format::FORMAT_16U2:
      return create_xyz_buffer<std::uint16_t>(data,
                                              xidx,
                                              yidx,
                                              zidx,
                                              width,
                                              height,
                                              fmt,
                                              mask);
    case pixel_format::FORMAT_16S:
      return create_xyz_buffer<std::int16_t>(data,
                                             xidx,
                                             yidx,
                                             zidx,
                                             width,
                                             height,
                                             fmt,
                                             mask);
    case pixel_format::FORMAT_32U:
      return create_xyz_buffer<std::uint32_t>(data,
                                              xidx,
                                              yidx,
                                              zidx,
                                              width,
                                              height,
                                              fmt,
                                              mask);
    case pixel_format::FORMAT_32S:
      return create_xyz_buffer<std::int32_t>(data,
                                             xidx,
                                             yidx,
                                             zidx,
                                             width,
                                             height,
                                             fmt,
                                             mask);
    case pixel_format::FORMAT_64U:
      return create_xyz_buffer<uint64_t>(data,
                                         xidx,
                                         yidx,
                                         zidx,
                                         width,
                                         height,
                                         fmt,
                                         mask);
    case pixel_format::FORMAT_32F:
    case pixel_format::FORMAT_32F3:
      return create_xyz_buffer<float>(data,
                                      xidx,
                                      yidx,
                                      zidx,
                                      width,
                                      height,
                                      fmt,
                                      mask);
    case pixel_format::FORMAT_64F:
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

std::map<ifm3d::image_chunk, std::set<std::size_t>>
ifm3d::get_image_chunks(const std::vector<std::uint8_t>& data,
                        std::size_t start_idx,
                        std::optional<size_t> end_idx)
{
  std::map<image_chunk, std::set<std::size_t>> chunks;

  std::size_t idx = start_idx; // start of first chunk
  std::size_t const size =
    (end_idx.has_value() ? end_idx.value() : data.size()) - 6;

  while (idx < size)
    {
      auto const chunk =
        static_cast<image_chunk>(mkval<std::uint32_t>(data.data() + idx));
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
  const std::map<image_chunk, std::set<std::size_t>>& chunks)
  -> decltype(chunks.end())
{
  // to get the metadata we use the confidence image for 3d and
  // the jpeg image for 2d

  auto metachunk = chunks.find(ifm3d::image_chunk::CONFIDENCE_IMAGE);

  if (metachunk == chunks.end())
    {
      metachunk = chunks.find(ifm3d::image_chunk::JPEG_IMAGE);
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

ifm3d::pixel_format
ifm3d::get_chunk_format(const std::vector<std::uint8_t>& data, std::size_t idx)
{
  return static_cast<pixel_format>(
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
    case ifm3d::pixel_format::FORMAT_8U:
    case ifm3d::pixel_format::FORMAT_8S:
      image.SetTo<std::uint8_t>(0, mask);
      break;

    case ifm3d::pixel_format::FORMAT_16U:
    case ifm3d::pixel_format::FORMAT_16S:
    case ifm3d::pixel_format::FORMAT_16U2:
      image.SetTo<std::uint16_t>(0, mask);
      break;

    case ifm3d::pixel_format::FORMAT_32U:
    case ifm3d::pixel_format::FORMAT_32S:
    case ifm3d::pixel_format::FORMAT_32F:
    case ifm3d::pixel_format::FORMAT_32F3:
      image.SetTo<std::uint32_t>(0, mask);
      break;

    case ifm3d::pixel_format::FORMAT_64U:
    case ifm3d::pixel_format::FORMAT_64F:
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
  Buffer mask = Buffer(confidence.Width(),
                       confidence.Height(),
                       1,
                       pixel_format::FORMAT_8U);

  int const index = 0;
  if (confidence.DataFormat() == pixel_format::FORMAT_16U)
    {
      std::transform(confidence.begin<std::uint16_t>(),
                     confidence.end<std::uint16_t>(),
                     mask.begin<std::uint8_t>(),
                     [](auto& value) -> uint8_t { return value & 0x1; });
    }
  else if (confidence.DataFormat() == pixel_format::FORMAT_8U)
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
  const std::map<ifm3d::image_chunk, std::set<std::size_t>>& chunks,
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
