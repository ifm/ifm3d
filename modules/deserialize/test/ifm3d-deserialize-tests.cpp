#include "test_utils.hpp"
#include <array>
#include <cstdint>
#include <gtest/gtest.h>
#include <ifm3d/common/err.h>
#include <ifm3d/deserialize/deserialize_o3d_buffers.hpp>
#include <ifm3d/deserialize/struct_imu_info_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_extrinsic_calibration_correction_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_info_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_occupancy_grid_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_polar_occupancy_grid_v1.hpp>
#include <ifm3d/deserialize/struct_rgb_info_v1.hpp>
#include <ifm3d/deserialize/struct_tof_info_v3.hpp>
#include <ifm3d/deserialize/struct_tof_info_v4.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3d.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/buffer_id.h>
#include <ifm3d/fg/frame_grabber.h>
#include <imu_info_test_data.hpp>
#include <memory>
#include <o3d_parameters.hpp>
#include <ods_extrinsic_calibration_correction.hpp>
#include <ods_info_test_data.hpp>
#include <ods_occupancy_grid.hpp>
#include <ods_polar_occupancy_grid.hpp>
#include <rgb_info_test_data.hpp>
#include <tof_info_test_data.hpp>

TEST(DeserializeTestWithFile, struct_tof_info_v3_size_exception)
{
  auto buffer = ifm3d::Buffer(1, 200, 1, ifm3d::PixelFormat::FORMAT_8U);

  EXPECT_THROW(ifm3d::TOFInfoV3::Deserialize(buffer), ifm3d::Error);
}
TEST(DeserializeTestWithFile, struct_tof_info_v3)
{
  auto buffer = ifm3d::read_buffer_from_file("tof_info.data");
  auto tof_info_v3 = ifm3d::TOFInfoV3::Deserialize(buffer);
  constexpr auto minimum_required_version = 3;
  EXPECT_GE(tof_info_v3.version, minimum_required_version);
  EXPECT_NEAR(tof_info_v3.amplitude_resolution,
              ifm3d::tof_info::AMPLITUDE_RESOLUTION,
              ifm3d::EPSILON);
  EXPECT_NEAR(tof_info_v3.distance_resolution,
              ifm3d::tof_info::DISTANCE_RESOLUTION,
              ifm3d::EPSILON);
  EXPECT_TRUE(
    ifm3d::compare_array(tof_info_v3.amp_normalization_factors,
                         ifm3d::tof_info::AMP_NORMALIZATION_FACTORS));

  std::array<float, 6> const extrinc_opt_to_user = {
    tof_info_v3.extrinsic_optic_to_user.trans_x,
    tof_info_v3.extrinsic_optic_to_user.trans_y,
    tof_info_v3.extrinsic_optic_to_user.trans_z,
    tof_info_v3.extrinsic_optic_to_user.rot_x,
    tof_info_v3.extrinsic_optic_to_user.rot_y,
    tof_info_v3.extrinsic_optic_to_user.rot_z};

  EXPECT_TRUE(ifm3d::compare_array(extrinc_opt_to_user,
                                   ifm3d::tof_info::EXTRINCSIC_OPTIC_TO_USER));

  EXPECT_EQ(tof_info_v3.intrinsic_calibration.model_id,
            ifm3d::tof_info::INTRINSIC_CALIB_MODEL_ID);

  EXPECT_TRUE(
    ifm3d::compare_array(tof_info_v3.intrinsic_calibration.model_parameters,
                         ifm3d::tof_info::INTRINSIC_CALIB_MODEL_PARAM));

  EXPECT_EQ(tof_info_v3.inverse_intrinsic_calibration.model_id,
            ifm3d::tof_info::INVERSE_INTRINSIC_CALIB_MODEL_ID);

  EXPECT_TRUE(ifm3d::compare_array(
    tof_info_v3.inverse_intrinsic_calibration.model_parameters,
    ifm3d::tof_info::INVERSE_INTRINSIC_CALIB_MODEL_PARAM));

  EXPECT_TRUE(ifm3d::compare_array(tof_info_v3.exposure_timestamps_ns,
                                   ifm3d::tof_info::EXPOSURE_TIMESTAMPS_NS));

  EXPECT_TRUE(ifm3d::compare_array(tof_info_v3.exposure_times_s,
                                   ifm3d::tof_info::EXPOSURE_TIMES_S));

  EXPECT_NEAR(tof_info_v3.illu_temperature,
              ifm3d::tof_info::ILLU_TEMPERATURE,
              ifm3d::EPSILON);

  EXPECT_TRUE(tof_info_v3.mode.compare(ifm3d::tof_info::MODE));

  EXPECT_TRUE(tof_info_v3.imager.compare(ifm3d::tof_info::IMAGER));
}

TEST(DeserializeTestWithFile, struct_tof_info_v4_size_exception)
{
  auto buffer = ifm3d::Buffer(1, 200, 1, ifm3d::PixelFormat::FORMAT_8U);

  EXPECT_THROW(ifm3d::TOFInfoV4::Deserialize(buffer), ifm3d::Error);
}
TEST(DeserializeTestWithFile, struct_tof_info_v4)
{

  auto buffer = ifm3d::read_buffer_from_file("tof_info.data");

  auto tof_info_v4 = ifm3d::TOFInfoV4::Deserialize(buffer);
  constexpr auto minimum_required_version = 4;
  EXPECT_GE(tof_info_v4.version, minimum_required_version);
  EXPECT_NEAR(tof_info_v4.amplitude_resolution,
              ifm3d::tof_info::AMPLITUDE_RESOLUTION,
              ifm3d::EPSILON);
  EXPECT_NEAR(tof_info_v4.distance_resolution,
              ifm3d::tof_info::DISTANCE_RESOLUTION,
              ifm3d::EPSILON);
  EXPECT_TRUE(
    ifm3d::compare_array(tof_info_v4.amp_normalization_factors,
                         ifm3d::tof_info::AMP_NORMALIZATION_FACTORS));

  std::array<float, 6> const extrinc_opt_to_user = {
    tof_info_v4.extrinsic_optic_to_user.trans_x,
    tof_info_v4.extrinsic_optic_to_user.trans_y,
    tof_info_v4.extrinsic_optic_to_user.trans_z,
    tof_info_v4.extrinsic_optic_to_user.rot_x,
    tof_info_v4.extrinsic_optic_to_user.rot_y,
    tof_info_v4.extrinsic_optic_to_user.rot_z};

  EXPECT_TRUE(ifm3d::compare_array(extrinc_opt_to_user,
                                   ifm3d::tof_info::EXTRINCSIC_OPTIC_TO_USER));

  EXPECT_EQ(tof_info_v4.intrinsic_calibration.model_id,
            ifm3d::tof_info::INTRINSIC_CALIB_MODEL_ID);

  EXPECT_TRUE(
    ifm3d::compare_array(tof_info_v4.intrinsic_calibration.model_parameters,
                         ifm3d::tof_info::INTRINSIC_CALIB_MODEL_PARAM));

  EXPECT_EQ(tof_info_v4.inverse_intrinsic_calibration.model_id,
            ifm3d::tof_info::INVERSE_INTRINSIC_CALIB_MODEL_ID);

  EXPECT_TRUE(ifm3d::compare_array(
    tof_info_v4.inverse_intrinsic_calibration.model_parameters,
    ifm3d::tof_info::INVERSE_INTRINSIC_CALIB_MODEL_PARAM));

  EXPECT_TRUE(ifm3d::compare_array(tof_info_v4.exposure_timestamps_ns,
                                   ifm3d::tof_info::EXPOSURE_TIMESTAMPS_NS));

  EXPECT_TRUE(ifm3d::compare_array(tof_info_v4.exposure_times_s,
                                   ifm3d::tof_info::EXPOSURE_TIMES_S));

  EXPECT_NEAR(tof_info_v4.illu_temperature,
              ifm3d::tof_info::ILLU_TEMPERATURE,
              ifm3d::EPSILON);

  EXPECT_TRUE(tof_info_v4.mode.compare(ifm3d::tof_info::MODE));

  EXPECT_TRUE(tof_info_v4.imager.compare(ifm3d::tof_info::IMAGER));

  EXPECT_EQ(tof_info_v4.measurement_block_index,
            ifm3d::tof_info::MEASUREMENT_BLOCK_INDEX);
  EXPECT_NEAR(tof_info_v4.measurement_range_min,
              ifm3d::tof_info::MEASUREMENT_RANGE_MIN,
              ifm3d::EPSILON);
  EXPECT_NEAR(tof_info_v4.measurement_range_max,
              ifm3d::tof_info::MEASUREMENT_RANGE_MAX,
              ifm3d::EPSILON);
}

TEST(DeserializeTestWithFile, struct_rgb_info_v1_size_exception)
{
  auto buffer = ifm3d::Buffer(1, 200, 1, ifm3d::PixelFormat::FORMAT_8U);

  EXPECT_THROW(ifm3d::RGBInfoV1::Deserialize(buffer), ifm3d::Error);
}
TEST(DeserializeTestWithFile, struct_rgb_info_v1)
{
  auto buffer = ifm3d::read_buffer_from_file("rgb_info.data");
  auto rgb_info_v1 = ifm3d::RGBInfoV1::Deserialize(buffer);

  constexpr auto minimum_required_version = 1;
  EXPECT_GE(rgb_info_v1.version, minimum_required_version);
  EXPECT_EQ(rgb_info_v1.frame_counter, ifm3d::rgb_info::FRAME_COUNTER);
  EXPECT_EQ(rgb_info_v1.timestamp_ns, ifm3d::rgb_info::TIMESTAMP_NS);
  EXPECT_NEAR(rgb_info_v1.exposure_time,
              ifm3d::rgb_info::EXPOSURE_TIME,
              ifm3d::EPSILON);

  std::array<float, 6> const extrinc_opt_to_user = {
    rgb_info_v1.extrinsic_optic_to_user.trans_x,
    rgb_info_v1.extrinsic_optic_to_user.trans_y,
    rgb_info_v1.extrinsic_optic_to_user.trans_z,
    rgb_info_v1.extrinsic_optic_to_user.rot_x,
    rgb_info_v1.extrinsic_optic_to_user.rot_y,
    rgb_info_v1.extrinsic_optic_to_user.rot_z};

  EXPECT_TRUE(ifm3d::compare_array(extrinc_opt_to_user,
                                   ifm3d::rgb_info::EXTRINCSIC_OPTIC_TO_USER));

  EXPECT_EQ(rgb_info_v1.intrinsic_calibration.model_id,
            ifm3d::rgb_info::INTRINSIC_CALIB_MODEL_ID);

  EXPECT_TRUE(
    ifm3d::compare_array(rgb_info_v1.intrinsic_calibration.model_parameters,
                         ifm3d::rgb_info::INTRINSIC_CALIB_MODEL_PARAM));

  EXPECT_EQ(rgb_info_v1.inverse_intrinsic_calibration.model_id,
            ifm3d::rgb_info::INVERSE_INTRINSIC_CALIB_MODEL_ID);

  EXPECT_TRUE(ifm3d::compare_array(
    rgb_info_v1.inverse_intrinsic_calibration.model_parameters,
    ifm3d::rgb_info::INVERSE_INTRINSIC_CALIB_MODEL_PARAM));
}

TEST(DeserializeTestWithFile, struct_ods_info_v1_size_exception)
{
  auto buffer = ifm3d::Buffer(1, 5, 1, ifm3d::PixelFormat::FORMAT_8U);

  EXPECT_THROW(ifm3d::ODSInfoV1::Deserialize(buffer), ifm3d::Error);
}

TEST(DeserializeTestWithFile, struct_ods_info_v1)
{
  auto buffer = ifm3d::read_buffer_from_file("o3r_ods_info.data");
  auto ods_info_v1 = ifm3d::ODSInfoV1::Deserialize(buffer);

  EXPECT_EQ(ods_info_v1.timestamp_ns, ifm3d::ods_info::TIMESTAMP_NS);
  EXPECT_EQ(ods_info_v1.zone_config_id, ifm3d::ods_info::ZONE_CONFIG_ID);
  ifm3d::compare_array<uint8_t, 3>(ods_info_v1.zone_occupied,
                                   ifm3d::ods_info::ZONE_OCCUPIED);
}

TEST(DeserializeTestWithFile, struct_ods_occupancy_grid_v1_size_exception)
{
  auto buffer = ifm3d::Buffer(1, 5, 1, ifm3d::PixelFormat::FORMAT_8U);

  EXPECT_THROW(ifm3d::ODSOccupancyGridV1::Deserialize(buffer), ifm3d::Error);
}

TEST(DeserializeTestWithFile, struct_ods_occupancy_grid_info_v1)
{
  auto buffer = ifm3d::read_buffer_from_file("o3r_ods_occupancy_grid.data");
  auto ods_occupancy_grid_v1 = ifm3d::ODSOccupancyGridV1::Deserialize(buffer);

  EXPECT_EQ(ods_occupancy_grid_v1.timestamp_ns,
            ifm3d::ods_occupancy_grid::TIMESTAMP_NS);
  EXPECT_EQ(ods_occupancy_grid_v1.width, ifm3d::ods_occupancy_grid::WIDTH);
  EXPECT_EQ(ods_occupancy_grid_v1.height, ifm3d::ods_occupancy_grid::HEIGHT);

  ifm3d::compare_array<float, 6>(
    ods_occupancy_grid_v1.transform_cell_center_to_user,
    ifm3d::ods_occupancy_grid::TRANSFORM_CELL_CENTER_TO_USER);

  EXPECT_EQ(ods_occupancy_grid_v1.image.Width(),
            ifm3d::ods_occupancy_grid::WIDTH);
  EXPECT_EQ(ods_occupancy_grid_v1.image.Height(),
            ifm3d::ods_occupancy_grid::HEIGHT);
  EXPECT_EQ(ods_occupancy_grid_v1.image.DataFormat(),
            ifm3d::PixelFormat::FORMAT_8U);
}

TEST(DeserializeTestWithDevice, struct_tof_info_v3)
{
  auto dev = std::make_shared<ifm3d::O3R>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(dev, 50012);

  fg->Start({ifm3d::buffer_id::TOF_INFO});
  auto frame = fg->WaitForFrame().get();

  auto buffer = frame->GetBuffer(ifm3d::buffer_id::TOF_INFO);
  EXPECT_NO_THROW(ifm3d::TOFInfoV3::Deserialize(buffer));
}

TEST(DeserializeTestWithDevice, struct_tof_info_v4)
{
  auto dev = std::make_shared<ifm3d::O3R>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(dev, 50012);

  fg->Start({ifm3d::buffer_id::TOF_INFO});
  auto frame = fg->WaitForFrame().get();

  auto buffer = frame->GetBuffer(ifm3d::buffer_id::TOF_INFO);

  EXPECT_NO_THROW(ifm3d::TOFInfoV3::Deserialize(buffer));
  EXPECT_NO_THROW(ifm3d::TOFInfoV4::Deserialize(buffer));
}

TEST(DeserializeTestWithDevice, struct_rgb_info_v1)
{
  auto dev = std::make_shared<ifm3d::O3R>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(dev, 50010);

  fg->Start({ifm3d::buffer_id::RGB_INFO});
  auto frame = fg->WaitForFrame().get();

  auto buffer = frame->GetBuffer(ifm3d::buffer_id::RGB_INFO);

  EXPECT_NO_THROW(ifm3d::RGBInfoV1::Deserialize(buffer));
}

TEST(DeserializeTestWithFile, struct_o3d_intrinsic_parameter)
{

  auto buffer = ifm3d::read_buffer_from_file("o3d_instrinsic.data");

  auto o3d_instrinsic_param =
    ifm3d::O3DInstrinsicCalibration::Deserialize(buffer);

  EXPECT_TRUE(ifm3d::compare_array(o3d_instrinsic_param.data,
                                   ifm3d::o3d::INTRINSIC_PARAMETER));

  ifm3d::Buffer const buffer_blank;
  EXPECT_THROW(ifm3d::O3DInstrinsicCalibration::Deserialize(buffer_blank),
               ifm3d::Error);
}

TEST(DeserializeTestWithFile, struct_o3d_inverse_intrinsic_parameter)
{

  auto buffer = ifm3d::read_buffer_from_file("o3d_inverse_instrinsic.data");

  auto o3d_param = ifm3d::O3DInverseInstrinsicCalibration::Deserialize(buffer);

  EXPECT_TRUE(ifm3d::compare_array(o3d_param.data,
                                   ifm3d::o3d::INVERS_INTRINSIC_PARAMETER));

  ifm3d::Buffer const buffer_blank;
  EXPECT_THROW(
    ifm3d::O3DInverseInstrinsicCalibration::Deserialize(buffer_blank),
    ifm3d::Error);
}

TEST(DeserializeTestWithFile, struct_o3d_extrinsic_parameter)
{

  auto buffer = ifm3d::read_buffer_from_file("o3d_extrinsic.data");

  auto o3d_param = ifm3d::O3DExtrinsicCalibration::Deserialize(buffer);

  EXPECT_TRUE(ifm3d::compare_array(o3d_param.data,
                                   ifm3d::o3d::EXTRINCSIC_CALIB_PARAMETER));

  ifm3d::Buffer const buffer_blank;
  EXPECT_THROW(ifm3d::O3DExtrinsicCalibration::Deserialize(buffer_blank),
               ifm3d::Error);
}

TEST(DeserializeTestWithFile, struct_o3d_exposure_time_parameter)
{

  auto buffer = ifm3d::read_buffer_from_file("o3d_exposure_times.data");

  auto o3d_param = ifm3d::O3DExposureTimes::Deserialize(buffer);

  EXPECT_TRUE(
    ifm3d::compare_array(o3d_param.data, ifm3d::o3d::EXPOSURE_TIMES));

  ifm3d::Buffer const buffer_blank;
  EXPECT_THROW(ifm3d::O3DExposureTimes::Deserialize(buffer_blank),
               ifm3d::Error);
}

TEST(DeserializeTestWithFile, struct_o3d_illu_temp_parameter)
{

  auto buffer = ifm3d::read_buffer_from_file("o3d_illu_temp.data");

  auto o3d_param = ifm3d::O3DIlluTemperature::Deserialize(buffer);

  EXPECT_TRUE(ifm3d::compare_array(o3d_param.data, ifm3d::o3d::ILLU_TEMP));

  ifm3d::Buffer const buffer_blank;
  EXPECT_THROW(ifm3d::O3DIlluTemperature::Deserialize(buffer_blank),
               ifm3d::Error);
}

TEST(DeserializeTestWithO3D, struct_o3d_intrinsic_parameter)
{

  auto dev = std::make_shared<ifm3d::O3D>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(dev, 50010);

  fg->Start({ifm3d::buffer_id::INTRINSIC_CALIB});
  auto frame = fg->WaitForFrame().get();

  auto buffer = frame->GetBuffer(ifm3d::buffer_id::INTRINSIC_CALIB);

  EXPECT_NO_THROW(ifm3d::O3DInstrinsicCalibration::Deserialize(buffer));
}

TEST(DeserializeTestWithO3D, struct_o3d_inverse_intrinsic_parameter)
{

  auto dev = std::make_shared<ifm3d::O3D>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(dev, 50010);

  fg->Start({ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION});
  auto frame = fg->WaitForFrame().get();

  auto buffer =
    frame->GetBuffer(ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION);

  EXPECT_NO_THROW(ifm3d::O3DInverseInstrinsicCalibration::Deserialize(buffer));
}

TEST(DeserializeTestWithO3D, struct_o3d_Extrinsic_parameter)
{

  auto dev = std::make_shared<ifm3d::O3D>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(dev, 50010);

  fg->Start({ifm3d::buffer_id::EXTRINSIC_CALIB});
  auto frame = fg->WaitForFrame().get();

  auto buffer = frame->GetBuffer(ifm3d::buffer_id::EXTRINSIC_CALIB);

  EXPECT_NO_THROW(ifm3d::O3DExtrinsicCalibration::Deserialize(buffer));
}

TEST(DeserializeTestWithO3D, struct_o3d_Exp_time_parameter)
{

  auto dev = std::make_shared<ifm3d::O3D>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(dev, 50010);

  fg->Start({ifm3d::buffer_id::EXPOSURE_TIME});
  auto frame = fg->WaitForFrame().get();

  auto buffer = frame->GetBuffer(ifm3d::buffer_id::EXPOSURE_TIME);

  EXPECT_NO_THROW(ifm3d::O3DExposureTimes::Deserialize(buffer));
}

TEST(DeserializeTestWithO3D, struct_o3d_illu_temp_parameter)
{

  auto dev = std::make_shared<ifm3d::O3D>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(dev, 50010);

  fg->Start({ifm3d::buffer_id::ILLUMINATION_TEMP});
  auto frame = fg->WaitForFrame().get();

  auto buffer = frame->GetBuffer(ifm3d::buffer_id::ILLUMINATION_TEMP);

  EXPECT_NO_THROW(ifm3d::O3DIlluTemperature::Deserialize(buffer));
}

TEST(DeserializeTestWithFile,
     struct_ods_polar_occupancy_grid_v1_size_exception)
{
  auto buffer = ifm3d::Buffer(1, 5, 1, ifm3d::PixelFormat::FORMAT_8U);

  EXPECT_THROW(ifm3d::ODSPolarOccupancyGridV1::Deserialize(buffer),
               ifm3d::Error);
}

TEST(DeserializeTestWithFile, struct_ods_polar_occupancy_grid_info_v1)
{
  auto buffer =
    ifm3d::read_buffer_from_file("o3r_ods_polar_occupancy_grid.data");
  auto ods_polar_occupancy_grid_v1 =
    ifm3d::ODSPolarOccupancyGridV1::Deserialize(buffer);

  EXPECT_EQ(ods_polar_occupancy_grid_v1.version,
            ifm3d::ods_polar_occupancy_grid::VERSION);

  ifm3d::compare_array<uint16_t, 675>(
    ods_polar_occupancy_grid_v1.polar_occ_grid,
    ifm3d::ods_polar_occupancy_grid::POLAR_OCC_GRID);

  EXPECT_EQ(ods_polar_occupancy_grid_v1.timestamp_ns,
            ifm3d::ods_polar_occupancy_grid::TIMESTAMP_NS);
}

TEST(DeserializeTestWithFile,
     struct_ods_extrinsic_calibration_correction_v1_size_exception)
{
  auto buffer = ifm3d::Buffer(1, 5, 1, ifm3d::PixelFormat::FORMAT_8U);

  EXPECT_THROW(ifm3d::ODSExtrinsicCalibrationCorrectionV1::Deserialize(buffer),
               ifm3d::Error);
}

TEST(DeserializeTestWithDevice, struct_ods_extrinsic_calibration_correction_v1)
{
  auto dev = std::make_shared<ifm3d::O3R>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(dev, 51010);

  fg->Start({ifm3d::buffer_id::O3R_ODS_EXTRINSIC_CALIBRATION_CORRECTION});
  auto frame = fg->WaitForFrame().get();

  auto buffer = frame->GetBuffer(
    ifm3d::buffer_id::O3R_ODS_EXTRINSIC_CALIBRATION_CORRECTION);

  EXPECT_NO_THROW(
    ifm3d::ODSExtrinsicCalibrationCorrectionV1::Deserialize(buffer));
}

TEST(DeserializeTestWithFile, struct_ods_extrinsic_calibration_correction_v1)
{
  auto buffer = ifm3d::read_buffer_from_file(
    "o3r_ods_extrinsic_calibration_correction.data");
  auto ods_extrinsic_calibration_correction_v1 =
    ifm3d::ODSExtrinsicCalibrationCorrectionV1::Deserialize(buffer);

  EXPECT_EQ(ods_extrinsic_calibration_correction_v1.version,
            ifm3d::ods_extrinsic_calibration_correction::VERSION);

  EXPECT_EQ(ods_extrinsic_calibration_correction_v1.completion_rate,
            ifm3d::ods_extrinsic_calibration_correction::COMPLETION_RATE);

  ifm3d::compare_array<float, 3>(
    ods_extrinsic_calibration_correction_v1.rot_delta_value,
    ifm3d::ods_extrinsic_calibration_correction::ROT_DELTA_VALUE);

  ifm3d::compare_array<uint8_t, 3>(
    ods_extrinsic_calibration_correction_v1.rot_delta_valid,
    ifm3d::ods_extrinsic_calibration_correction::ROT_DELTA_VALID);

  ifm3d::compare_array<float, 3>(
    ods_extrinsic_calibration_correction_v1.rot_head_to_user,
    ifm3d::ods_extrinsic_calibration_correction::ROT_HEAD_TO_USER);
}

TEST(DeserializeTestWithFile, struct_imu_info_v1_size_exception)
{
  auto buffer = ifm3d::Buffer(1, 5, 1, ifm3d::PixelFormat::FORMAT_8U);

  EXPECT_THROW(ifm3d::IMUInfoV1::Deserialize(buffer), ifm3d::Error);
}

TEST(DeserializeTestWithDevice, struct_imu_info_v1)
{
  auto dev = std::make_shared<ifm3d::O3R>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(dev, 50016);

  fg->Start({ifm3d::buffer_id::O3R_RESULT_IMU});
  auto frame = fg->WaitForFrame().get();

  auto buffer = frame->GetBuffer(ifm3d::buffer_id::O3R_RESULT_IMU);

  EXPECT_NO_THROW(ifm3d::IMUInfoV1::Deserialize(buffer));
}

TEST(DeserializeTestWithFile, struct_imu_info_v1)
{
  auto buffer = ifm3d::read_buffer_from_file("imu_info.data");
  auto imu_info_v1 = ifm3d::IMUInfoV1::Deserialize(buffer);

  EXPECT_GE(imu_info_v1.imu_version, ifm3d::imu_info::IMU_VERSION);

  EXPECT_EQ(imu_info_v1.num_samples, ifm3d::imu_info::NUM_SAMPLES);

  for (std::uint32_t i = 0; i < imu_info_v1.num_samples; ++i)
    {
      const auto& actual = imu_info_v1.imu_samples[i];
      const auto& expect = ifm3d::imu_info::IMU_SAMPLES[i];

      EXPECT_EQ(actual.hw_timestamp, expect.hw_timestamp);
      EXPECT_EQ(actual.timestamp, expect.timestamp);

      constexpr float k_tol = 1e-5F;
      EXPECT_NEAR(actual.temperature, expect.temperature, k_tol);
      EXPECT_NEAR(actual.acc_x, expect.acc_x, k_tol);
      EXPECT_NEAR(actual.acc_y, expect.acc_y, k_tol);
      EXPECT_NEAR(actual.acc_z, expect.acc_z, k_tol);
      EXPECT_NEAR(actual.gyro_x, expect.gyro_x, k_tol);
      EXPECT_NEAR(actual.gyro_y, expect.gyro_y, k_tol);
      EXPECT_NEAR(actual.gyro_z, expect.gyro_z, k_tol);
    }

  std::array<float, 6> const extrinc_imu_to_user = {
    imu_info_v1.extrinsic_imu_to_user.trans_x,
    imu_info_v1.extrinsic_imu_to_user.trans_y,
    imu_info_v1.extrinsic_imu_to_user.trans_z,
    imu_info_v1.extrinsic_imu_to_user.rot_x,
    imu_info_v1.extrinsic_imu_to_user.rot_y,
    imu_info_v1.extrinsic_imu_to_user.rot_z};
  EXPECT_TRUE(ifm3d::compare_array(extrinc_imu_to_user,
                                   ifm3d::imu_info::EXTRINSIC_IMU_TO_USER));

  std::array<float, 6> const extrinc_imu_to_vpu = {
    imu_info_v1.extrinsic_imu_to_vpu.trans_x,
    imu_info_v1.extrinsic_imu_to_vpu.trans_y,
    imu_info_v1.extrinsic_imu_to_vpu.trans_z,
    imu_info_v1.extrinsic_imu_to_vpu.rot_x,
    imu_info_v1.extrinsic_imu_to_vpu.rot_y,
    imu_info_v1.extrinsic_imu_to_vpu.rot_z};
  EXPECT_TRUE(ifm3d::compare_array(extrinc_imu_to_vpu,
                                   ifm3d::imu_info::EXTRINSIC_IMU_TO_VPU));

  EXPECT_EQ(imu_info_v1.imu_fifo_rcv_timestamp,
            ifm3d::imu_info::IMU_FIFO_RCV_TIMESTAMP_NS);
}