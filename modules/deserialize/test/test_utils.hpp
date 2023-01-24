/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_TEST_UTILS_HPP
#define IFM3D_DESERIALIZE_TEST_UTILS_HPP

namespace ifm3d
{
  constexpr auto epsilon = 1e-6;
  template <typename T>
  bool
  compare(const T& a, const T& b)
  {
    return a == b;
  }
#if 1
  template <>
  bool
  compare<float>(const float& a, const float& b)
  {
    if (std::fabs(a - b) > epsilon)
      std::cout << a << " " << b << " " << std::fabs(a - b) << std::endl;
    return std::fabs(a - b) < epsilon;
  }
#endif
  // this is copied from buffer.hpp
  // remove this after addition of size function in buffer

  template <typename T, size_t n>
  bool
  compare_array(const std::array<T, n>& actual,
                const std::array<T, n>& expected)
  {
    return std::equal(
      actual.begin(),
      actual.end(),
      expected.begin(),
      [](const T& a, const T& b) { return ifm3d::compare(a, b); });
  }

  template <typename T, size_t n>
  void
  print_array(const std::array<T, n>& arr)
  {
    std::cout << "{ ";
    for (auto& val : arr)
      {
        std::cout << std::setprecision(10) << static_cast<T>(val) << ", ";
      }
    std::cout << "};";
  }

  void
  write_buffer_to_file(const ifm3d::Buffer& buffer, std::string file_name)
  {
    const uint32_t width = buffer.width();
    const uint32_t height = buffer.height();
    const uint32_t nchannel = buffer.nchannels();
    const ifm3d::pixel_format pix_format = buffer.dataFormat();

    auto buffer_file =
      std::fstream(file_name, std::ios::out | std::ios::binary);

    buffer_file.write(reinterpret_cast<const char*>(&width), sizeof(width));
    buffer_file.write(reinterpret_cast<const char*>(&height), sizeof(height));

    buffer_file.write(reinterpret_cast<const char*>(&nchannel),
                      sizeof(nchannel));
    buffer_file.write(reinterpret_cast<const char*>(&pix_format),
                      sizeof(pixel_format));

    buffer_file.write(reinterpret_cast<const char*>(buffer.ptr<uint8_t>(0)),
                      buffer.size());
    buffer_file.close();
  }

  ifm3d::Buffer
  read_buffer_from_file(std::string file_name)
  {
    auto buffer_file = std::ifstream(file_name, std::ios::binary);
    uint32_t width;
    uint32_t height;
    uint32_t nchannel;
    ifm3d::pixel_format pix_format;

    std::vector<std::uint8_t> fileBuffer;
    std::istreambuf_iterator<char> iter(buffer_file);
    std::copy(iter,
              std::istreambuf_iterator<char>(),
              std::back_inserter(fileBuffer));
    buffer_file.close();
    width = ifm3d::mkval<uint32_t>(fileBuffer.data());
    height = ifm3d::mkval<uint32_t>(fileBuffer.data() + 4);
    nchannel = ifm3d::mkval<uint32_t>(fileBuffer.data() + 8);
    pix_format = static_cast<ifm3d::pixel_format>(
      ifm3d::mkval<uint32_t>(fileBuffer.data() + 12));

    auto buffer = ifm3d::Buffer(width, height, nchannel, pix_format);
    std::copy(fileBuffer.begin() + 16, fileBuffer.end(), buffer.begin<char>());

    return buffer;
  }
};

#endif // IFM3D_DESERIALIZE_TEST_UTILS_HPP