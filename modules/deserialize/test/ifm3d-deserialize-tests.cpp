#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include "gtest/gtest.h"
#include <ifm3d/deserialize.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/fg.h>
#include <ifm3d/deserialize/struct_tof_info_v3.hpp>
#include <algorithm>
#include <fstream>
#include <tof_info_test_data.hpp>
#include <limits>

namespace ifm3d
{
  constexpr auto epsilon = 1e-6;
  template <typename T>
  bool
  compare(const T& a, const T& b)
  {
    return a == b;
  }

  template <>
  bool
  compare<float>(const float& a, const float& b)
  {
    if (std::fabs(a - b) > epsilon)
      std::cout << a << " " << b << " " << std::fabs(a - b) << std::endl;
    return std::fabs(a - b) < epsilon;
  }

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
        std::cout << std::setprecision(10) << val << ", ";
      }
    std::cout << "};";
  }

  std::unordered_map<uint32_t, std::size_t> PIX_SZ{
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U), 1},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S), 1},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U), 2},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S), 2},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S), 4},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F), 4},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F), 8},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U2), 2},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F3), 4}};

  void
  write_buffer_to_file(const ifm3d::Buffer& buffer, std::string file_name)
  {
    const uint32_t width = buffer.width();
    const uint32_t height = buffer.height();
    const uint32_t nchannel = buffer.nchannels();
    const ifm3d::pixel_format pix_format = buffer.dataFormat();

    auto buffer_file = std::fstream(file_name, std::ios::out);

    buffer_file.write(reinterpret_cast<const char*>(&width), sizeof(width));
    buffer_file.write(reinterpret_cast<const char*>(&height), sizeof(height));

    buffer_file.write(reinterpret_cast<const char*>(&nchannel),
                      sizeof(nchannel));
    buffer_file.write(reinterpret_cast<const char*>(&pix_format),
                      sizeof(pixel_format));

    buffer_file.write(reinterpret_cast<const char*>(buffer.ptr<uint8_t>(0)),
                      (width * height * nchannel *
                       PIX_SZ[static_cast<std::uint32_t>(pix_format)]));
  }

  ifm3d::Buffer
  read_buffer_from_file(std::string file_name)
  {
    auto buffer_file = std::fstream(file_name, std::ios::in);
    uint32_t width;
    uint32_t height;
    uint32_t nchannel;
    ifm3d::pixel_format pix_format;

    buffer_file.read(reinterpret_cast<char*>(&width), sizeof(width));
    buffer_file.read(reinterpret_cast<char*>(&height), sizeof(height));

    buffer_file.read(reinterpret_cast<char*>(&nchannel), sizeof(nchannel));
    buffer_file.read(reinterpret_cast<char*>(&pix_format),
                     sizeof(pixel_format));

    auto buffer = ifm3d::Buffer(width,
                                height,
                                nchannel,
                                static_cast<ifm3d::pixel_format>(pix_format));

    buffer_file.read(reinterpret_cast<char*>(buffer.ptr<uint8_t>(0)),
                     (width * height * nchannel *
                      PIX_SZ[static_cast<std::uint32_t>(pix_format)]));

    return buffer;
  }
};

class DeserializeTest : public ::testing::Test
{
protected:
  virtual void
  SetUp()
  {}

  virtual void
  TearDown()
  {}
};

TEST(DeserializeTestWithFile, struct_tof_info_v3)
{

  auto buffer = ifm3d::read_buffer_from_file("tof_info.data");

  auto tof_info_v3 = ifm3d::TofInfoV3::Deserialize(buffer);

  EXPECT_GE(tof_info_v3.version, ifm3d::tof_info_v3::version);
  EXPECT_NEAR(tof_info_v3.amplitude_resolution,
              ifm3d::tof_info_v3::amplitude_resolution,
              ifm3d::epsilon);
  EXPECT_NEAR(tof_info_v3.distance_resolution,
              ifm3d::tof_info_v3::distance_resolution,
              ifm3d::epsilon);
  EXPECT_TRUE(
    ifm3d::compare_array(tof_info_v3.amp_normalization_factors,
                         ifm3d::tof_info_v3::amp_normalization_factors));

  std::array<float, 6> extrinc_opt_to_user = {
    tof_info_v3.extrisic_optic_to_user.transX,
    tof_info_v3.extrisic_optic_to_user.transY,
    tof_info_v3.extrisic_optic_to_user.transZ,
    tof_info_v3.extrisic_optic_to_user.rotX,
    tof_info_v3.extrisic_optic_to_user.rotY,
    tof_info_v3.extrisic_optic_to_user.rotZ};

  EXPECT_TRUE(
    ifm3d::compare_array(extrinc_opt_to_user,
                         ifm3d::tof_info_v3::extrincsic_optic_to_user));

  EXPECT_EQ(tof_info_v3.intrinsic_calibration.modelID,
            ifm3d::tof_info_v3::intrinsic_calib_model_id);

  EXPECT_TRUE(
    ifm3d::compare_array(tof_info_v3.intrinsic_calibration.modelParameters,
                         ifm3d::tof_info_v3::intrinsic_calib_model_param));

  EXPECT_EQ(tof_info_v3.inverse_intrinsic_calibration.modelID,
            ifm3d::tof_info_v3::inverse_intrinsic_calib_model_id);

  EXPECT_TRUE(ifm3d::compare_array(
    tof_info_v3.inverse_intrinsic_calibration.modelParameters,
    ifm3d::tof_info_v3::inverse_intrinsic_calib_model_param));

  EXPECT_TRUE(
    ifm3d::compare_array(tof_info_v3.exposure_timestamps_ns,
                         ifm3d::tof_info_v3::exposure_timestamps_ns));

  EXPECT_TRUE(ifm3d::compare_array(tof_info_v3.exposure_times_s,
                                   ifm3d::tof_info_v3::exposure_times_s));

  EXPECT_NEAR(tof_info_v3.illu_temperature,
              ifm3d::tof_info_v3::illu_temperature,
              ifm3d::epsilon);

  EXPECT_TRUE(
    ifm3d::compare_array(tof_info_v3.mode, ifm3d::tof_info_v3::mode));

  EXPECT_TRUE(
    ifm3d::compare_array(tof_info_v3.imager, ifm3d::tof_info_v3::imager));
}
