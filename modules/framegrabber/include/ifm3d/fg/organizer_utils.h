/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_ORGANIZER_UTILS_H
#define IFM3D_FG_ORGANIZER_UTILS_H

#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/distance_image_info.h>
#include <ifm3d/fg/frame.h>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <vector>

namespace ifm3d
{
  constexpr std::size_t IMG_BUFF_START = 8;

  std::map<ImageChunk, std::set<std::size_t>> get_image_chunks(
    const std::vector<std::uint8_t>& data,
    std::size_t start_idx,
    std::optional<size_t> end_idx = std::nullopt);

  std::size_t get_format_size(PixelFormat fmt);
  std::size_t get_format_channels(PixelFormat fmt);

  Buffer create_1d_buffer(const std::vector<std::uint8_t>& data,
                          std::size_t idx);

  Buffer create_buffer(const std::vector<std::uint8_t>& data,
                       std::size_t idx,
                       std::size_t width,
                       std::size_t height);

  Buffer create_buffer(const std::vector<std::uint8_t>& data,
                       std::size_t idx,
                       std::size_t width,
                       std::size_t height,
                       PixelFormat fmt,
                       const std::optional<json>& metadata = std::nullopt);

  Buffer create_xyz_buffer(const std::vector<std::uint8_t>& data,
                           std::size_t xidx,
                           std::size_t yidx,
                           std::size_t zidx,
                           std::size_t width,
                           std::size_t height,
                           PixelFormat fmt,
                           const std::optional<Buffer>& mask);

  auto find_metadata_chunk(
    const std::map<ImageChunk, std::set<std::size_t>>& chunks)
    -> decltype(chunks.end());

  std::tuple<uint32_t, uint32_t> get_image_size(
    const std::vector<std::uint8_t>& data,
    std::size_t idx);

  PixelFormat get_chunk_format(const std::vector<std::uint8_t>& data,
                               std::size_t idx);

  uint32_t get_chunk_frame_count(const std::vector<std::uint8_t>& data,
                                 std::size_t idx);

  std::vector<TimePointT> get_chunk_timestamps(
    const std::vector<uint8_t>& data,
    std::size_t idx);

  std::size_t get_chunk_pixeldata_offset(const std::vector<std::uint8_t>& data,
                                         std::size_t idx);

  std::size_t get_chunk_size(const std::vector<std::uint8_t>& data,
                             std::size_t idx);

  std::size_t get_chunk_pixeldata_size(const std::vector<std::uint8_t>& data,
                                       std::size_t idx);

  std::size_t get_chunk_header_version(const std::vector<std::uint8_t>& data,
                                       std::size_t idx);

  void mask_buffer(Buffer& image, const Buffer& mask);

  bool is_probably_blob(const std::vector<std::uint8_t>& data,
                        std::size_t idx,
                        std::size_t width,
                        std::size_t height);

  Buffer create_pixel_mask(Buffer& confidence);

  void parse_data(
    const std::vector<uint8_t>& data,
    const std::set<buffer_id>& requested_images,
    const std::map<ifm3d::ImageChunk, std::set<std::size_t>>& chunks,
    size_t width,
    size_t height,
    std::map<buffer_id, BufferList>& data_blob,
    std::map<buffer_id, BufferList>& data_image);

  void mask_images(
    std::map<ifm3d::buffer_id, ifm3d::BufferList>& images,
    ifm3d::Buffer& mask,
    const std::function<bool(ifm3d::buffer_id id)>& should_mask);

  bool has_metadata(const std::vector<std::uint8_t>& data, std::size_t idx);

  ifm3d::json create_metadata(const std::vector<std::uint8_t>& data,
                              std::size_t idx);
  buffer_id map_metadata_to_buffer_id(const ifm3d::Buffer& buffer);

  std::map<ifm3d::buffer_id, ifm3d::BufferList> map_logical_buffers(
    const std::map<ifm3d::buffer_id, ifm3d::BufferList>& images,
    const std::set<ifm3d::buffer_id>& requested_images);

  /**
   * Create a value of type T from sizeof(T) bytes of the passed in byte
   * buffer. Given that the ifm sensors transmit data in little endian
   * format, this function will swap bytes if necessary for the host
   * representation of T.
   *
   * @param[in] buff A pointer to a buffer in memory intended to be
   * interpreted as data of type T and assuming the buffer is little endian.
   *
   * @return An interpretation of `buff` as type T with bytes swapped as
   * appropriate for the host's byte ordering semantics.
   */
  template <typename T>
  T
  mkval(const unsigned char* buff)
  {
    union
    {
      T v;
      unsigned char bytes[sizeof(T)]; // NOLINT(modernize-avoid-c-arrays)
    } value;

#if !defined(_WIN32) && __BYTE_ORDER == __BIG_ENDIAN
    std::reverse_copy(buff, buff + sizeof(T), value.bytes);
#else
    std::copy(buff, buff + sizeof(T), value.bytes);
#endif

    return value.v;
  }
  /**
   * @brief Helper function to convert std::vector<T> to 1XN buffer
   */
  template <typename T>
  ifm3d::Buffer
  create_buffer_from_vector(const std::vector<T>& vec)
  {
    ifm3d::Buffer buf =
      Buffer(vec.size(),
             1,
             ifm3d::FormatType<T>::NumChannels,
             static_cast<ifm3d::PixelFormat>(ifm3d::FormatType<T>::Format));
    std::copy(vec.begin(), vec.end(), buf.begin<T>());
    return buf;
  }
  /**
   * @brief Helper function to convert struct T to 1XN buffer
   */
  template <typename T>
  ifm3d::Buffer
  create_buffer_from_struct(const T& struct_object)
  {
    ifm3d::Buffer buf = Buffer(sizeof(T), 1, 1, ifm3d::PixelFormat::FORMAT_8U);
    const auto* start = reinterpret_cast<const uint8_t*>(&struct_object);
    auto* ptr = buf.Ptr<uint8_t>(0);
    std::copy(start, start + sizeof(T), ptr);
    return buf;
  }
  /**
   * @brief Helper function to convertifm3d buffer to struct T
   */
  template <typename T>
  T
  convert_buffer_to_struct(const ifm3d::Buffer& buf)
  {
    T struct_object;
    const auto* ptr = buf.Ptr<uint8_t>(0);
    std::copy(ptr,
              ptr + sizeof(T),
              reinterpret_cast<uint8_t*>(&struct_object));
    return struct_object;
  }
  /**
   * @brief Helper function to convertifm3d buffer
   */
  template <typename T>
  inline ifm3d::Buffer
  create_xyz_buffer(const std::vector<std::uint8_t>& data,
                    std::size_t xidx,
                    std::size_t yidx,
                    std::size_t zidx,
                    std::size_t width,
                    std::size_t height,
                    ifm3d::PixelFormat fmt,
                    const std::optional<ifm3d::Buffer>& mask)
  {
    std::size_t const incr = sizeof(T);
    std::size_t const npts = width * height;

    ifm3d::Buffer im(width, height, 3, fmt);

    int col = 0;
    int row = -1;
    int xyz_col = 0;

    T* xyz_ptr = NULL;
    T x;
    T y;
    T z;

    constexpr T bad_pixel = 0;

    for (std::size_t i = 0; i < npts;
         ++i, xidx += incr, yidx += incr, zidx += incr)
      {
        col = static_cast<int>(i % static_cast<size_t>(width));
        xyz_col = col * 3;
        if (col == 0)
          {
            row += 1;
            xyz_ptr = im.Ptr<T>(row);
          }

        x = ifm3d::mkval<T>(data.data() + xidx);
        y = ifm3d::mkval<T>(data.data() + yidx);
        z = ifm3d::mkval<T>(data.data() + zidx);

        if (mask.has_value() && mask.value().At<uint8_t>(row, col) != 0)
          {
            xyz_ptr[xyz_col] = bad_pixel;
            xyz_ptr[xyz_col + 1] = bad_pixel;
            xyz_ptr[xyz_col + 2] = bad_pixel;
          }
        else
          {
            xyz_ptr[xyz_col] = x;
            xyz_ptr[xyz_col + 1] = y;
            xyz_ptr[xyz_col + 2] = z;
          }
      }

    return im;
  }

} // end: namespace ifm3d

#endif // IFM3D_FG_ORGANIZER_UTILS_H