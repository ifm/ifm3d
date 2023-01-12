#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include "gtest/gtest.h"
#include <ifm3d/deserialize.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg.h>
#include <ifm3d/deserialize/struct_tof_info_v3.hpp>
#include <algorithm>
#include <fstream>
#include "tof_info_test_data.hpp"
#include "test_utils.hpp"
#include <limits>

TEST(DeserializeTestWithFile, structv3_size_exception)
{
  auto buffer = ifm3d::Buffer(1, 200, 1, ifm3d::pixel_format::FORMAT_8U);

  EXPECT_THROW(ifm3d::TofInfoV3::Deserialize(buffer), ifm3d::Error);
}
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
