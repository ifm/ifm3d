/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#  ifndef IFM3D_FG_ORGANIZER_UTILS_H
#    define IFM3D_FG_ORGANIZER_UTILS_H

#    include <map>
#    include <optional>
#    include <vector>
#    include <tuple>
#    include <ifm3d/device/device.h>
#    include <ifm3d/fg/buffer.h>
#    include <ifm3d/fg/frame.h>
#    include <ifm3d/fg/distance_image_info.h>

namespace ifm3d
{
  constexpr std::size_t IMG_BUFF_START = 8;

  std::map<image_chunk, std::size_t> get_image_chunks(
    const std::vector<std::uint8_t>& data,
    std::size_t start_idx);

  std::size_t get_format_size(pixel_format fmt);
  std::size_t get_format_channels(pixel_format fmt);

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
                       pixel_format fmt);

  Buffer create_xyz_buffer(const std::vector<std::uint8_t>& data,
                           std::size_t xidx,
                           std::size_t yidx,
                           std::size_t zidx,
                           std::size_t width,
                           std::size_t height,
                           pixel_format fmt,
                           const std::optional<Buffer>& mask);

  auto find_metadata_chunk(const std::map<image_chunk, std::size_t>& chunks)
    -> decltype(chunks.end());

  std::tuple<uint32_t, uint32_t> get_image_size(
    const std::vector<std::uint8_t>& data,
    std::size_t idx);

  pixel_format get_chunk_format(const std::vector<std::uint8_t>& data,
                                std::size_t idx);

  uint32_t get_chunk_frame_count(const std::vector<std::uint8_t>& data,
                                 std::size_t idx);

  std::vector<TimePointT> get_chunk_timestamps(
    const std::vector<uint8_t>& data,
    std::size_t idx);

  std::size_t get_chunk_pixeldata_offset(const std::vector<std::uint8_t>& data,
                                         std::size_t idx);

  std::size_t get_chunk_pixeldata_size(const std::vector<std::uint8_t>& data,
                                       std::size_t idx);

  void mask_buffer(Buffer& image, const Buffer& mask);

  bool is_probably_blob(const std::vector<std::uint8_t>& data,
                        std::size_t idx,
                        std::size_t width,
                        std::size_t height);

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
      unsigned char bytes[sizeof(T)];
    } value;

#    if !defined(_WIN32) && __BYTE_ORDER == __BIG_ENDIAN
    std::reverse_copy(buff, buff + sizeof(T), value.bytes);
#    else
    std::copy(buff, buff + sizeof(T), value.bytes);
#    endif

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
             ifm3d::FormatType<T>::nchannel,
             static_cast<ifm3d::pixel_format>(ifm3d::FormatType<T>::format));
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
    ifm3d::Buffer buf =
      Buffer(sizeof(T), 1, 1, ifm3d::pixel_format::FORMAT_8U);
    const uint8_t* start = reinterpret_cast<const uint8_t*>(&struct_object);
    auto ptr = buf.ptr<uint8_t>(0);
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
    auto ptr = buf.ptr<uint8_t>(0);
    std::copy(ptr,
              ptr + sizeof(T),
              reinterpret_cast<uint8_t*>(&struct_object));
    return struct_object;
  }

} // end: namespace ifm3d

#  endif // IFM3D_FG_ORGANIZER_UTILS_H
#endif   // DOXYGEN_SHOULD_SKIP_THIS